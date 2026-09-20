#include "train_pathing_recalculation.h"

#include "FGTrain.h"
#include "FGLocomotive.h"
#include "FGRailroadTimeTable.h"
#include "FGTrainStationIdentifier.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "RailroadNavigation.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "Patching/NativeHookManager.h"

DEFINE_LOG_CATEGORY(LogSeamlessTrainPathing);

namespace SeamlessTrainPathingConstants
{
	// Extra safety margin [s] added on top of the train's own brake distance to account for how
	// long the scheduler needs to drop the old reservations and approve the new ones (typically a
	// handful of ticks). As long as the repath is triggered earlier than "time to reach the signal
	// at current brake distance" the scheduler will have finished re-reserving the new blocks before
	// the train's brake distance actually starts overlapping the signal, so the signal never has to
	// flash red and the train never has to brake.
	static constexpr float SchedulerReactionTimeMargin = 1.5f;

	// Minimum lookahead distance [cm] regardless of speed/brake distance, so that trains standing
	// still or moving slowly still repath with some margin before reaching a signal or switch.
	static constexpr float MinLookaheadDistance = 2000.f;

	// Extra flat distance [cm] added on top of the brake distance derived lookahead, this accounts
	// for the fact that BrakeDistance is measured to a full stop, but we want to be recalculating
	// while still comfortably above the point the train would actually need to start braking.
	static constexpr float LookaheadSafetyDistance = 1000.f;

	// How often [s] we should retry pathing while stopped for a long time at a signal.
	static constexpr float WaitAtSignalRepathInterval = 4.f;
}

/** Per-train bookkeeping so we don't spam path requests for the same signal/switch. */
struct FTrainRepathState
{
	TWeakObjectPtr<UFGRailroadTrackConnectionComponent> LastSignalRepathedFor = nullptr;
	TWeakObjectPtr<UFGRailroadTrackConnectionComponent> LastSwitchRepathedFor = nullptr;
	int32 LastWaitRepathBucket = 0;
};

static TMap<TWeakObjectPtr<AFGTrain>, FTrainRepathState> GTrainRepathStates;

/** Periodically remove entries for trains that no longer exist. */
static void CleanupStaleRepathStates()
{
	for (auto It = GTrainRepathStates.CreateIterator(); It; ++It)
	{
		if (!It->Key.IsValid())
		{
			It.RemoveCurrent();
		}
	}
}

/**
 * Computes how far [cm] ahead of a signal/switch we should trigger a repath so that the scheduler
 * has enough time to drop old reservations and approve the new ones before the train gets close
 * enough that it would otherwise have to brake for a momentarily red/unreserved signal.
 *
 * This scales with the train's current speed and brake distance: A fast, heavy train needs much
 * more lookahead than one crawling along at low speed.
 */
static float ComputeLookaheadDistance(const AFGTrain* Train)
{
	const float Speed = FMath::Abs(Train->mAtcData.CurrentSpeed);
	const float BrakeDistance = Train->mAtcData.BrakeDistance;

	// Distance covered while the scheduler is busy re-evaluating reservations.
	const float SchedulerReactionDistance = Speed * SeamlessTrainPathingConstants::SchedulerReactionTimeMargin;

	const float Lookahead = BrakeDistance + SchedulerReactionDistance + SeamlessTrainPathingConstants::LookaheadSafetyDistance;

	return FMath::Max(Lookahead, SeamlessTrainPathingConstants::MinLookaheadDistance);
}

/**
 * Checks whether the freshly found path is effectively identical to the remaining portion of the
 * train's currently active path, i.e. the train has not been re-routed at all - it is simply
 * further along the same route than it was when the old path was found.
 *
 * We compare by track connection identity: Starting at the train's current path segment in the old
 * path, each subsequent connection must match the corresponding connection in the new path one for
 * one. The new path is expected to be the same length or shorter (since the train may have already
 * progressed past some points), never longer, and every point it does contain must line up exactly.
 *
 * If this returns true, swapping in the new path would be a no-op change to the route and can be
 * skipped entirely, so the scheduler is never bothered with reservations it doesn't actually need
 * to touch, avoiding the momentary visual/physical hitches that come from needlessly dropping and
 * re-approving otherwise still-valid reservations.
 */
static bool IsSameRemainingPath(const FRailroadPath& OldPath, int32 OldPathCurrentSegment, const FRailroadPath& NewPath)
{
	if (OldPath.Station != NewPath.Station)
	{
		return false;
	}

	if (!OldPath.PathPoints.IsValidIndex(OldPathCurrentSegment))
	{
		return false;
	}

	const int32 RemainingOldPointCount = OldPath.PathPoints.Num() - OldPathCurrentSegment;

	// The new path (found from further along the route) can never legitimately contain more
	// remaining points than the old path had left - if it does, something about the route itself
	// changed (e.g. a different, longer detour was chosen), so treat it as a real change.
	if (NewPath.PathPoints.Num() > RemainingOldPointCount)
	{
		return false;
	}

	for (int32 NewIndex = 0; NewIndex < NewPath.PathPoints.Num(); ++NewIndex)
	{
		const int32 OldIndex = OldPathCurrentSegment + NewIndex;

		UFGRailroadTrackConnectionComponent* OldConnection = OldPath.PathPoints[OldIndex].TrackConnection.Get();
		UFGRailroadTrackConnectionComponent* NewConnection = NewPath.PathPoints[NewIndex].TrackConnection.Get();

		if (OldConnection != NewConnection)
		{
			return false;
		}
	}

	return true;
}

/**
 * Tries to find and seamlessly swap in a fresh path for the given self driving train.
 *
 * The new path is only ever applied via the normal FTrainAtcData::SetPath(), which lets the
 * scheduler correctly drop reservations for blocks that are no longer needed and (re-)evaluate
 * the ones that are. What makes this seamless is not skipping that re-evaluation, but triggering
 * it early enough - while the train is still further away from the next signal/switch than its
 * current brake distance plus a safety margin for the scheduler's reaction time - so that by the
 * time the train's brake distance would start overlapping the signal, the scheduler has already
 * finished approving the new reservations and the signal is green again.
 */
static void TryRepathTrainSeamlessly(AFGTrain* Train)
{
	if (!Train || !Train->HasAuthority())
	{
		return;
	}

	if (!Train->IsSelfDrivingEnabled() || !Train->HasTimeTable())
	{
		return;
	}

	if (Train->GetTrainStatus() != ETrainStatus::TS_SelfDriving)
	{
		return;
	}

	static int32 CleanupCounter = 0;
	if (++CleanupCounter >= 256)
	{
		CleanupCounter = 0;
		CleanupStaleRepathStates();
	}

	FTrainRepathState& State = GTrainRepathStates.FindOrAdd(Train);

	const float LookaheadDistance = ComputeLookaheadDistance(Train);

	bool bShouldRepath = false;

	// About to pass a signal - trigger while still further away than our dynamic lookahead so the
	// scheduler finishes re-reserving before we'd actually need to brake for it.
	if (UFGRailroadTrackConnectionComponent* NextSignal = Train->mAtcData.NextSignalConnection.Get())
	{
		if (Train->mAtcData.NextSignalDistance <= LookaheadDistance &&
			State.LastSignalRepathedFor.Get() != NextSignal)
		{
			bShouldRepath = true;
			State.LastSignalRepathedFor = NextSignal;
		}
	}

	// About to enter a track split (switch) - same idea as for signals.
	if (UFGRailroadTrackConnectionComponent* NextSwitch = Train->mAtcData.NextSwitchConnection.Get())
	{
		if (Train->mAtcData.NextSwitchDistance <= LookaheadDistance &&
			State.LastSwitchRepathedFor.Get() != NextSwitch)
		{
			bShouldRepath = true;
			State.LastSwitchRepathedFor = NextSwitch;
		}
	}

	// Standing for a longer time at a signal, keep retrying periodically. There is no "approach"
	// concern here since the train is already stationary, so no extra lookahead margin is needed.
	const float WaitTime = Train->mSelfDrivingData.TimeWaitingAtSignal;
	if (WaitTime > 0.f)
	{
		const int32 WaitBucket = FMath::FloorToInt(WaitTime / SeamlessTrainPathingConstants::WaitAtSignalRepathInterval);
		if (WaitBucket > State.LastWaitRepathBucket)
		{
			bShouldRepath = true;
			State.LastWaitRepathBucket = WaitBucket;
		}
	}
	else
	{
		State.LastWaitRepathBucket = 0;
	}

	if (!bShouldRepath)
	{
		return;
	}

	AFGLocomotive* Locomotive = Train->GetMultipleUnitMaster();
	if (!Locomotive)
	{
		return;
	}

	AFGRailroadTimeTable* TimeTable = Train->GetTimeTable();
	if (!TimeTable)
	{
		return;
	}

	const int32 CurrentStopIndex = TimeTable->GetCurrentStop();
	if (!TimeTable->IsValidStop(CurrentStopIndex))
	{
		return;
	}

	const FTimeTableStop Stop = TimeTable->GetStop(CurrentStopIndex);
	if (!Stop.Station)
	{
		return;
	}

	AFGBuildableRailroadStation* Station = Stop.Station->GetStation();
	if (!Station)
	{
		return;
	}

	FRailroadGraphAStarFilter Filter;
	const FRailroadPathFindingResult NewResult = FRailroadNavigation::FindPathSync(Locomotive, Station, Filter);

	// Only swap the plan if the new path is fully valid - this guarantees the train never ends up
	// without a path as a result of this mod. Since we triggered this well ahead of the next signal
	// / switch, the scheduler has ample time to drop obsolete reservations and approve the new ones
	// (turning the relevant signals green) before the train gets close enough to need to brake.
	if (NewResult.Result != ERailroadPathFindingResult::RPFR_Success)
	{
		return;
	}

	// If the newly found path is identical to what is left of the currently active path (just
	// possibly shorter, since the train progressed further along it), there is nothing to actually
	// change about the route. Skip the swap entirely so the scheduler's existing reservations for
	// the unchanged blocks/signals ahead are left completely untouched.
	if (Train->mAtcData.HasPath() && Train->mAtcData.Path.IsValid() && NewResult.Path.IsValid() &&
		IsSameRemainingPath(*Train->mAtcData.Path, Train->mAtcData.CurrentPathSegment, *NewResult.Path))
	{
		UE_LOG(LogSeamlessTrainPathing, Verbose, TEXT("Skipped redundant repath for train '%s', remaining route is unchanged."),
			*Train->GetTrainName().ToString());
		return;
	}

	Train->mAtcData.SetPath(NewResult);

	UE_LOG(LogSeamlessTrainPathing, Verbose, TEXT("Recalculated path for train '%s' %.0fcm ahead of next signal/switch (lookahead was %.0fcm)."),
		*Train->GetTrainName().ToString(), FMath::Min(Train->mAtcData.NextSignalDistance, Train->mAtcData.NextSwitchDistance), LookaheadDistance);
}

void FSeamlessTrainPathingModule::Startup()
{
	SUBSCRIBE_METHOD_VIRTUAL_AFTER(AFGTrain::Tick, GetMutableDefault<AFGTrain>(), [](AFGTrain* Train, float dt)
		{
			TryRepathTrainSeamlessly(Train);
		});
}

void FSeamlessTrainPathingModule::Shutdown()
{
}
