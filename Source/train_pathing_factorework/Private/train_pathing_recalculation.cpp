#include "train_pathing_recalculation.h"

#include "FGTrain.h"
#include "FGLocomotive.h"
#include "FGRailroadTimeTable.h"
#include "FGTrainStationIdentifier.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "Buildables/FGBuildableRailroadSignal.h"
#include "RailroadNavigation.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "Patching/NativeHookManager.h"
#include "train_pathing_factorework.h"

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
	static constexpr float LookaheadSafetyDistance = 1500.f;

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
 * Finds the index (within OldPath.PathPoints, at or after StartSearchIndex) of the first path point
 * whose connection is guarded by a signal that is safe to use as a splice point - i.e. a signal that
 * is not currently showing Clear. If a signal ahead is already Clear, the scheduler has likely
 * already committed a reservation through it (or into the path block chain beyond it), so we must
 * not re-plan starting there - doing so could clip a reservation the train is already relying on to
 * keep another signal further back green. Instead, we skip past any such signal and keep looking for
 * the next one down the line that has not yet been granted.
 *
 * @return Index into OldPath.PathPoints to splice at, or INDEX_NONE if no safe splice point exists
 *         before the end of the path (in which case a full repath from the front should be used).
 */
static int32 FindSafeSplicePathIndex(const FRailroadPath& OldPath, int32 StartSearchIndex)
{
	for (int32 Index = StartSearchIndex; Index < OldPath.PathPoints.Num(); ++Index)
	{
		UFGRailroadTrackConnectionComponent* Connection = OldPath.PathPoints[Index].TrackConnection.Get();
		if (!IsValid(Connection))
		{
			continue;
		}

		AFGBuildableRailroadSignal* Signal = Connection->GetFacingSignal();
		if (!IsValid(Signal))
		{
			continue;
		}

		// A signal that is already Clear likely has an active reservation backing it (either its
		// own block reservation, or - for path signals - a reservation chain extending further
		// down the line). Splicing here would replace path points the scheduler may still depend
		// on to keep that reservation valid, so skip past it and look further ahead instead.
		if (Signal->GetAspect() == ERailroadSignalAspect::RSA_Clear)
		{
			continue;
		}

		return Index;
	}

	return INDEX_NONE;
}

/**
 * Attempts a "spliced" repath: Instead of re-planning the entire route from the train's current
 * position, this keeps the untouched prefix of the currently active path (everything up to and
 * including the next signal that does not yet have an active Clear reservation) completely intact,
 * and only asks the pathfinder to plan the portion from that signal onward to the destination.
 *
 * This has two benefits over a full repath:
 *  1. The prefix - which the train is already committed to and which may already be reserved - is
 *     never re-evaluated by A*, so it can never be affected by transient cost changes (e.g. another
 *     train momentarily occupying a track further down the SAME prefix due to how
 *     CountVehiclesOnTrack factors in current occupancy). This removes a source of the scheduler
 *     seeing a "different" path for a section that, in reality, cannot change anymore.
 *  2. The search space for A* is smaller, since it starts further along the route.
 *
 * @return true if a spliced path was found and applied (or found to be unchanged and skipped);
 *         false if no safe splice point was available or the search failed, in which case the
 *         caller should fall back to a full repath from the train's current position.
 */
static bool TrySpliceRepathFromNextSignal(AFGTrain* Train, AFGLocomotive* Locomotive, AFGBuildableRailroadStation* Station)
{
	if (!Train->mAtcData.HasPath() || !Train->mAtcData.Path.IsValid())
	{
		return false;
	}

	const FRailroadPath& OldPath = *Train->mAtcData.Path;
	const int32 CurrentSegment = FMath::Max(Train->mAtcData.CurrentPathSegment, 0);

	const int32 SpliceIndex = FindSafeSplicePathIndex(OldPath, CurrentSegment);
	if (SpliceIndex == INDEX_NONE || !OldPath.PathPoints.IsValidIndex(SpliceIndex))
	{
		// No safe splice point found ahead (e.g. entire remaining route is one long chain of
		// already-clear path signals) - fall back to a full repath.
		return false;
	}

	UFGRailroadTrackConnectionComponent* SpliceConnection = OldPath.PathPoints[SpliceIndex].TrackConnection.Get();
	if (!IsValid(SpliceConnection))
	{
		return false;
	}

	FRailroadGraphAStarFilter Filter;

	// The splice connection is itself already part of the existing (unreserved-beyond-this-point)
	// path, so it must be included as the first point of the new suffix, matching how the regular
	// search includes the locomotive's own starting connection.
	const FRailroadPathFindingResult SuffixResult = FindPathSyncFrom(
		Locomotive,
		SpliceConnection,
		/*bIgnoredStart=*/true,
		Station,
		Filter);

	if (SuffixResult.Result != ERailroadPathFindingResult::RPFR_Success || !SuffixResult.Path.IsValid())
	{
		return false;
	}

	// Splice: everything before SpliceIndex is carried over untouched from the old path, the rest
	// is replaced by the freshly planned suffix.
	FRailroadPathFindingResult SplicedResult;
	SplicedResult.Locomotive = Locomotive;
	SplicedResult.Goal = SuffixResult.Goal;
	SplicedResult.Result = ERailroadPathFindingResult::RPFR_Success;
	SplicedResult.Path = MakeShared<FRailroadPath>();
	SplicedResult.Path->Station = Station;
	SplicedResult.Path->PathPoints.Reserve(SpliceIndex + SuffixResult.Path->PathPoints.Num());

	for (int32 Index = 0; Index < SpliceIndex; ++Index)
	{
		SplicedResult.Path->PathPoints.Add(OldPath.PathPoints[Index]);
	}

	// Recompute distances for the carried-over prefix so they correctly lead into the new suffix'
	// distances (the old prefix distances were relative to the old path's own goal).
	const int32 SuffixPointCount = SuffixResult.Path->PathPoints.Num();
	for (const FRailroadPathPoint& SuffixPoint : SuffixResult.Path->PathPoints)
	{
		SplicedResult.Path->PathPoints.Add(SuffixPoint);
	}

	{
		const int32 NumPts = SplicedResult.Path->PathPoints.Num();
		if (NumPts > 0)
		{
			SplicedResult.Path->PathPoints[NumPts - 1].Distance = SplicedResult.Path->PathPoints[NumPts - 1].Distance;

			for (int32 Index = NumPts - 2; Index >= 0; --Index)
			{
				// Only need to recompute the prefix distances (>= suffix start), the suffix's own
				// distances are already correct relative to its own goal.
				if (Index >= SpliceIndex)
				{
					continue;
				}

				UFGRailroadTrackConnectionComponent* ConnA = SplicedResult.Path->PathPoints[Index].TrackConnection.Get();
				UFGRailroadTrackConnectionComponent* ConnB = SplicedResult.Path->PathPoints[Index + 1].TrackConnection.Get();
				float SegmentLen = 0.0f;

				if (ConnA && ConnB)
				{
					AFGBuildableRailroadTrack* TrackB = ConnB->GetTrack();
					SegmentLen = TrackB
						? TrackB->GetLength()
						: FVector::Dist(ConnA->GetComponentLocation(), ConnB->GetComponentLocation());
				}

				SplicedResult.Path->PathPoints[Index].Distance = SplicedResult.Path->PathPoints[Index + 1].Distance + SegmentLen;
			}
		}
	}

	// If, after splicing, the remaining route is identical to what's already active, skip applying
	// it entirely - same reasoning as the full-repath case: never touch reservations needlessly.
	if (IsSameRemainingPath(OldPath, CurrentSegment, *SplicedResult.Path))
	{
		UE_LOG(LogSeamlessTrainPathing, Verbose,
			TEXT("Skipped redundant spliced repath for train '%s', remaining route (from next non-clear signal) is unchanged."),
			*Train->GetTrainName().ToString());
		return true;
	}

	Train->mAtcData.SetPath(SplicedResult);

	UE_LOG(LogSeamlessTrainPathing, Verbose,
		TEXT("Spliced repath for train '%s': kept %d untouched path point(s), replanned %d point(s) from next non-clear signal onward."),
		*Train->GetTrainName().ToString(), SpliceIndex, SuffixPointCount);

	return true;
}

/**
 * Attempts a full repath (from the locomotive's current position to its next timetable stop),
 * used as a fallback whenever a spliced repath isn't possible (e.g. no safe splice point found).
 */
static bool TryFullRepath(AFGTrain* Train, AFGLocomotive* Locomotive, AFGBuildableRailroadStation* Station)
{
	UFGRailroadTrackConnectionComponent* LocForward = Locomotive->GetTrackPosition().GetForwardConnection();
	UFGRailroadTrackConnectionComponent* StartConnection = LocForward
		? LocForward
		: Locomotive->GetTrackPosition().GetReverseConnection();

	if (!IsValid(StartConnection))
	{
		return false;
	}

	FRailroadGraphAStarFilter Filter;
	const FRailroadPathFindingResult Result = FindPathSyncFrom(
		Locomotive,
		StartConnection,
		/*bIgnoredStart=*/true,
		Station,
		Filter);

	if (Result.Result != ERailroadPathFindingResult::RPFR_Success || !Result.Path.IsValid())
	{
		return false;
	}

	Train->mAtcData.SetPath(Result);
	return true;
}

/**
 * Periodically re-evaluates self-driving trains' paths ahead of upcoming signals/switches so the
 * route to the next non-clear signal is refreshed early enough for the scheduler to re-approve
 * reservations before the train would otherwise have to brake for a momentarily unreserved signal.
 *
 * This is throttled per-train (via GTrainRepathStates) so that we only trigger a repath once per
 * upcoming signal/switch, rather than every tick while approaching it.
 */
static void TryRepathTrainSeamlessly(AFGTrain* Train)
{
	if (!IsValid(Train) || Train->IsDerailed())
	{
		return;
	}

	if (!Train->IsSelfDrivingEnabled() || Train->GetTrainStatus() != ETrainStatus::TS_SelfDriving)
	{
		return;
	}

	AFGLocomotive* Locomotive = Train->GetMultipleUnitMaster();
	if (!IsValid(Locomotive))
	{
		return;
	}

	AFGRailroadTimeTable* TimeTable = Train->GetTimeTable();
	if (!IsValid(TimeTable) || TimeTable->GetNumStops() == 0)
	{
		return;
	}

	const FTimeTableStop Stop = TimeTable->GetStop(TimeTable->GetCurrentStop());
	if (!IsValid(Stop.Station))
	{
		return;
	}

	AFGBuildableRailroadStation* Station = Stop.Station->GetStation();
	if (!IsValid(Station))
	{
		return;
	}

	if (!Train->mAtcData.HasPath() || !Train->mAtcData.Path.IsValid())
	{
		return;
	}

	const float LookaheadDistance = ComputeLookaheadDistance(Train);

	UFGRailroadTrackConnectionComponent* NextSignalConnection = Train->mAtcData.NextSignalConnection.Get();
	UFGRailroadTrackConnectionComponent* NextSwitchConnection = Train->mAtcData.NextSwitchConnection.Get();

	const bool bApproachingSignal = IsValid(NextSignalConnection) &&
		Train->mAtcData.NextSignalDistance <= LookaheadDistance;

	const bool bApproachingSwitch = IsValid(NextSwitchConnection) &&
		Train->mAtcData.NextSwitchDistance <= LookaheadDistance;

	FTrainRepathState& RepathState = GTrainRepathStates.FindOrAdd(Train);

	bool bShouldRepath = false;

	if (bApproachingSignal && RepathState.LastSignalRepathedFor.Get() != NextSignalConnection)
	{
		bShouldRepath = true;
		RepathState.LastSignalRepathedFor = NextSignalConnection;
	}
	
	if (bApproachingSwitch && RepathState.LastSwitchRepathedFor.Get() != NextSwitchConnection)
	{
		bShouldRepath = true;
		RepathState.LastSwitchRepathedFor = NextSwitchConnection;
	}

	// If the train has been sitting still waiting at a signal for a while, periodically retry
	// pathing in case the situation ahead has changed (e.g. another train has since cleared out).
	if (!bShouldRepath && Train->mSelfDrivingData.TimeWaitingAtSignal > 0.f)
	{
		const int32 WaitBucket = FMath::FloorToInt(
			Train->mSelfDrivingData.TimeWaitingAtSignal / SeamlessTrainPathingConstants::WaitAtSignalRepathInterval);

		if (WaitBucket > RepathState.LastWaitRepathBucket)
		{
			bShouldRepath = true;
			RepathState.LastWaitRepathBucket = WaitBucket;
		}
	}
	else if (Train->mSelfDrivingData.TimeWaitingAtSignal <= 0.f)
	{
		RepathState.LastWaitRepathBucket = 0;
	}

	if (!bShouldRepath)
	{
		return;
	}

	CleanupStaleRepathStates();

	// Prefer the spliced repath (only replans the portion beyond the next non-clear signal), and
	// only fall back to a full repath from the locomotive's own position if that isn't possible.
	if (TrySpliceRepathFromNextSignal(Train, Locomotive, Station))
	{
		return;
	}

	TryFullRepath(Train, Locomotive, Station);
}

void FSeamlessTrainPathingModule::Startup()
{
	SUBSCRIBE_METHOD(FTrainAtcData::SetPath, [](auto& Scope, FTrainAtcData* Self, const FRailroadPathFindingResult& Result)
		{
			if (Result.Result == ERailroadPathFindingResult::RPFR_Success &&
				Result.Path.IsValid() &&
				Self->HasPath() &&
				Self->Path.IsValid() &&
				IsSameRemainingPath(*Self->Path, Self->CurrentPathSegment, *Result.Path))
			{
				UE_LOG(LogSeamlessTrainPathing, Verbose,
					TEXT("Suppressed SetPath call: new path is identical to the remaining active route, avoiding needless reservation churn."));

				// Report success without ever touching Self->Path, so the scheduler never sees a "new"
				// path object and therefore never cancels/re-creates the still-valid reservations.
				Scope.Override(true);
				return;
			}
			else {
				Scope(Self, Result);
			}

			// Genuine route change (or an invalid/partial result) - let it proceed as normal so the
			// scheduler can correctly re-reserve the parts of the route that actually changed.
		});
	SUBSCRIBE_METHOD_VIRTUAL_AFTER(AFGTrain::Tick, GetMutableDefault<AFGTrain>(), [](AFGTrain* Train, float dt)
		{
			TryRepathTrainSeamlessly(Train);
		});
}

void FSeamlessTrainPathingModule::Shutdown()
{
}
