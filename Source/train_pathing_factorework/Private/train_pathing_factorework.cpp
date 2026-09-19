// Copyright Epic Games, Inc. All Rights Reserved.

#include "train_pathing_factorework.h"
#include "TrainPathingConfigStruct.h"
#include "train_pathing_debug.h"

#include "Patching/NativeHookManager.h"

#include "RailroadNavigation.h"
#include "FGLocomotive.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "Buildables/FGBuildableTrainPlatform.h"
#include "FGTrainPlatformConnection.h"
#include "GraphAStar.h"
#include "Buildables/FGBuildableTrainPlatformCargo.h"
#include "Buildables/FGBuildableTrainPlatformEmpty.h"
#include "FGFreightWagon.h"

DEFINE_LOG_CATEGORY(train_pathing);

float CountVehiclesOnTrack(
    AFGBuildableRailroadTrack* Track,
    const FTrainPathingConfigStruct& Config,
    const AFGTrain* IgnoredTrain)
{
    float Counts = 0.0f;

    if (!IsValid(Track))
    {
        return Counts;
    }
    bool bHasNonIgnoredVehicle = false;
    for (const TObjectPtr<AFGRailroadVehicle>& Vehicle :
        Track->GetVehicles())
    {
        AFGRailroadVehicle* VehicleActor = Vehicle.Get();

        if (!IsValid(VehicleActor))
        {
            continue;
        }
        if (IsValid(IgnoredTrain) &&
            VehicleActor->GetTrain() == IgnoredTrain)
        {
            continue;
        }
        bHasNonIgnoredVehicle = true;
        if (AFGLocomotive* Locomotive =
            Cast<AFGLocomotive>(VehicleActor))
        {
            if (Locomotive->IsOrientationReversed())
            {
                Counts += Config.Trains.LocomotiveReversedPenalty;
            }
            else
            {
                Counts += Config.Trains.LocomotiveForwardPenalty;
            }

            AFGTrain* Train = Locomotive->GetTrain();

            if (IsValid(Train))
            {
                if (Train->IsPlayerDriven())
                {
                    Counts += Config.Trains.PlayerDrivenTrainPenalty;
                }

                switch (Train->GetSelfDrivingError())
                {
                    case ESelfDrivingLocomotiveError::SDLE_NoTimeTable:
                        Counts += Config.Trains.SelfDriving.NoTimeTablePenalty;
                        break;

                    case ESelfDrivingLocomotiveError::SDLE_InvalidNextStop:
                        Counts += Config.Trains.SelfDriving.InvalidNextStopPenalty;
                        break;

                    case ESelfDrivingLocomotiveError::SDLE_InvalidLocomotivePlacement:
                        Counts += Config.Trains.SelfDriving.InvalidLocomotivePenalty;
                        break;

                    case ESelfDrivingLocomotiveError::SDLE_NoPath:
                        Counts += Config.Trains.SelfDriving.NoPathPenalty;
                        break;

                    case ESelfDrivingLocomotiveError::SDLE_StationUnreachable:
                        Counts += Config.Trains.SelfDriving.StationUnreachablePenalty;
                        break;

                    case ESelfDrivingLocomotiveError::SDLE_StationUnreachableWithSignals:
                        Counts += Config.Trains.SelfDriving.SignalUnreachablePenalty;
                        break;

                    case ESelfDrivingLocomotiveError::SDLE_LongWaitAtSignal:
                        Counts += Config.Trains.SelfDriving.LongWaitPenalty;
                        break;

                    case ESelfDrivingLocomotiveError::SDLE_NoError:
                    default:
                        break;
                }

                switch (Train->GetDockingState())
                {
                    case ETrainDockingState::TDS_ReadyToDock:
                        Counts += Config.Platforms.Docking.ReadyPenalty;
                        break;

                    case ETrainDockingState::TDS_Docked:
                        Counts += Config.Platforms.Docking.CompletePenalty;
                        break;

                    case ETrainDockingState::TDS_None:
                    default:
                        break;
                }
            }
        }
        else if (Cast<AFGFreightWagon>(VehicleActor))
        {
            Counts += Config.Trains.FreightWagonPenalty;
        }

        if (VehicleActor->IsDocked())
        {
            Counts += Config.Trains.DockedVehiclePenalty;
        }

        if (VehicleActor->IsDerailed())
        {
            Counts += Config.Trains.DerailedVehiclePenalty;
        }
    }
    if (bHasNonIgnoredVehicle)
    {
        Counts += Track->GetLength() /
            Config.Other.BasePenaltyScale;
    }
    return Counts;
}

float CountStationPlatforms(
    UFGRailroadTrackConnectionComponent* RailroadConnection,
    const FTrainPathingConfigStruct& Config)
{
    float Counts = 0.0f;

    if (!IsValid(RailroadConnection))
    {
        return Counts;
    }

    AFGBuildableRailroadStation* Station =
        RailroadConnection->GetStation();

    if (!IsValid(Station))
    {
        return Counts;
    }

    Counts+= Config.Platforms.StationBasePenalty;

    UFGTrainPlatformConnection* CurrentConnection =
        Station->GetStationOutputConnection();

    if (!IsValid(CurrentConnection))
    {
        return Counts;
    }

    TSet<UFGTrainPlatformConnection*> VisitedConnections;

    while (CurrentConnection)
    {
        if (VisitedConnections.Contains(CurrentConnection))
        {
            break;
        }

        VisitedConnections.Add(CurrentConnection);

        // Die aktuelle Verbindung zeigt auf die nächste Plattform.
        UFGTrainPlatformConnection* ConnectedPlatformConnection =
            CurrentConnection->GetConnectedTo();

        if (!IsValid(ConnectedPlatformConnection))
        {
            break;
        }

        AFGBuildableTrainPlatform* Platform =
            ConnectedPlatformConnection->GetPlatformOwner();

        if (!IsValid(Platform))
        {
            break;
        }

        if (Cast<AFGBuildableTrainPlatformCargo>(Platform))
        {
            switch (Cast<AFGBuildableTrainPlatformCargo>(Platform)->GetDockingStatus()) {
                case ETrainPlatformDockingStatus::ETPDS_WaitingToStart:
                    Counts += Config.Platforms.CargoPlatform.WaitingPenalty;
                    break;

                case ETrainPlatformDockingStatus::ETPDS_Loading:
                case ETrainPlatformDockingStatus::ETPDS_Unloading:
                    Counts += Config.Platforms.CargoPlatform.LoadingPenalty;
                    break;

                case ETrainPlatformDockingStatus::ETPDS_WaitingForTransfer:
                    Counts += Config.Platforms.CargoPlatform.TransferPenalty;
                    break;

                case ETrainPlatformDockingStatus::ETPDS_Complete:
                    Counts += Config.Platforms.CargoPlatform.CompletePenalty;
                    break;

                case ETrainPlatformDockingStatus::ETPDS_WaitForTransferCondition:
                    Counts += Config.Platforms.CargoPlatform.ConditionPenalty;
                    break;

                case ETrainPlatformDockingStatus::ETPDS_IdleWaitForTime:
                    Counts += Config.Platforms.CargoPlatform.IdlePenalty;
                    break;

                case ETrainPlatformDockingStatus::ETPDS_None:
                default:
                    break;
            }
        }
        else if (Cast<AFGBuildableTrainPlatformEmpty>(Platform))
        {
            Counts += Config.Platforms.EmptyPlatformPenalty;
        }
        else {
            Counts += 0.0f;
        }

        /*
         * Eine Plattform besitzt zwei Verbindungen. Nachdem wir auf der
         * Plattform angekommen sind, wechseln wir zur gegenüberliegenden
         * Verbindung und folgen anschließend der nächsten Plattform.
         */
        UFGTrainPlatformConnection* OppositeConnection =
            Platform->GetConnectionInOppositeDirection(
                ConnectedPlatformConnection);

        if (!IsValid(OppositeConnection))
        {
            break;
        }

        CurrentConnection = OppositeConnection;
    }

    return Counts;
}


FFactorioRailroadAStarFilter::FFactorioRailroadAStarFilter(
    const FRailroadGraphAStarFilter& InBase,
    const FTrainPathingConfigStruct& InConfig)
    : BaseFilter(InBase)
    , Config(InConfig)
{
}

float FFactorioRailroadAStarFilter::GetHeuristicScale() const
{
    return BaseFilter.GetHeuristicScale();
}

float FFactorioRailroadAStarFilter::GetHeuristicCost(
    const FRailroadGraphAStarPathPoint& StartNodeRef,
    const FRailroadGraphAStarPathPoint& EndNodeRef) const
{
    float OrigCost = BaseFilter.GetHeuristicCost(StartNodeRef, EndNodeRef);

    if (!IsValid(StartNodeRef.TrackConnection) || !IsValid(EndNodeRef.TrackConnection))
    {
        return 0.0f;
    }
    float NewCost = FVector::Dist(StartNodeRef.TrackConnection->GetComponentLocation(), EndNodeRef.TrackConnection->GetComponentLocation());
    //UE_LOG(train_pathing, Verbose, TEXT("Heuristic = %f <=> %f"), OrigCost, NewCost);
    return NewCost;
}

bool FFactorioRailroadAStarFilter::IsTraversalAllowed(
    const FRailroadGraphAStarPathPoint& NodeA,
    const FRailroadGraphAStarPathPoint& NodeB) const
{
    bool bAllowed = BaseFilter.IsTraversalAllowed(NodeA, NodeB);
    //UE_LOG(train_pathing, Verbose, TEXT("Orig_IsTraversal = %d"), bAllowed ? 1 : 0);

    if (!IsValid(NodeA.TrackConnection) || !IsValid(NodeB.TrackConnection))
    {
        return false;
    }

    return bAllowed;
}

#include "FGRailroadSubsystem.h"

bool DoesTrainPathContainTrack(
    const AFGTrain* Train,
    const AFGBuildableRailroadTrack* Track,
    const AFGTrain* IgnoredTrain)
{
    if (!IsValid(Train) ||
        !IsValid(Track) ||
        Train == IgnoredTrain ||
        !Train->mAtcData.Path.IsValid())
    {
        return false;
    }

    const TArray<FRailroadPathPoint>& PathPoints =
        Train->mAtcData.Path->PathPoints;

    if (PathPoints.Num() == 0)
    {
        return false;
    }

    int32 FirstUnpassedPathPoint =
        Train->mAtcData.CurrentPathSegment;

    /*
     * CurrentPathSegment kann während der Initialisierung INDEX_NONE
     * sein. In diesem Fall ist der Fortschritt noch unbekannt und der
     * gesamte Pfad wird berücksichtigt.
     */
    if (FirstUnpassedPathPoint == INDEX_NONE)
    {
        FirstUnpassedPathPoint = 0;
    }

    FirstUnpassedPathPoint = FMath::Clamp(
        FirstUnpassedPathPoint,
        0,
        PathPoints.Num() - 1
    );

    for (int32 PathPointIndex = FirstUnpassedPathPoint;
        PathPointIndex < PathPoints.Num();
        ++PathPointIndex)
    {
        UFGRailroadTrackConnectionComponent* Connection =
            PathPoints[PathPointIndex].TrackConnection.Get();

        if (IsValid(Connection) &&
            Connection->GetTrack() == Track)
        {
            return true;
        }
    }

    return false;
}

bool IsTrackInAnyTrainPath(
    const AFGBuildableRailroadTrack* Track,
    const AFGTrain* IgnoredTrain)
{
    if (!IsValid(Track))
    {
        return false;
    }

    AFGRailroadSubsystem* RailroadSubsystem =
        AFGRailroadSubsystem::Get(Track->GetWorld());

    if (!IsValid(RailroadSubsystem))
    {
        return false;
    }

    TArray<AFGTrain*> Trains;
    RailroadSubsystem->GetAllTrains(Trains);

    for (AFGTrain* Train : Trains)
    {
        if (!IsValid(Train) ||
            Train == IgnoredTrain ||
            Train->GetTrackGraphID() != Track->GetTrackGraphID())
        {
            continue;
        }

        if (DoesTrainPathContainTrack(Train, Track,
            IgnoredTrain))
        {
            return true;
        }
    }

    return false;
}

namespace
{
    float CalculateTrackGeometryPenalty(
        const AFGBuildableRailroadTrack* Track,
        const FTrainPathingConfigStruct& Config)
    {
        if (!IsValid(Track))
        {
            return 0.0f;
        }

        USplineComponent* Spline = Track->GetSplineComponent();

        if (!IsValid(Spline))
        {
            return 0.0f;
        }

        const float SplineLength = Spline->GetSplineLength();

        if (SplineLength <= KINDA_SMALL_NUMBER)
        {
            return 0.0f;
        }

        constexpr float SampleDistance = 250.0f;
        const int32 SampleCount = FMath::Max(
            1,
            FMath::CeilToInt(SplineLength / SampleDistance));

        const float DistanceStep = SplineLength /
            static_cast<float>(SampleCount);

        float Penalty = 0.0f;

        for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
        {
            const float StartDistance =
                static_cast<float>(SampleIndex) * DistanceStep;
            const float EndDistance =
                static_cast<float>(SampleIndex + 1) * DistanceStep;

            const FVector StartLocation =
                Spline->GetLocationAtDistanceAlongSpline(
                    StartDistance,
                    ESplineCoordinateSpace::World);

            const FVector EndLocation =
                Spline->GetLocationAtDistanceAlongSpline(
                    EndDistance,
                    ESplineCoordinateSpace::World);

            const FVector StartTangent =
                Spline->GetTangentAtDistanceAlongSpline(
                    StartDistance,
                    ESplineCoordinateSpace::World).GetSafeNormal();

            const FVector EndTangent =
                Spline->GetTangentAtDistanceAlongSpline(
                    EndDistance,
                    ESplineCoordinateSpace::World).GetSafeNormal();

            const float HorizontalDistance = FVector2D(
                EndLocation.X - StartLocation.X,
                EndLocation.Y - StartLocation.Y).Size();

            if (HorizontalDistance > KINDA_SMALL_NUMBER)
            {
                const float Slope =
                    (EndLocation.Z - StartLocation.Z) /
                    HorizontalDistance;

                if (Slope > Config.Tracks.Thresholds.ClimbingSlopeThreshold)
                {
                    const float SlopeIntensity =
                        (Slope - Config.Tracks.Thresholds.ClimbingSlopeThreshold) /
                        FMath::Max(
                            Config.Tracks.Thresholds.ClimbingSlopeThreshold,
                            KINDA_SMALL_NUMBER);

                    Penalty +=
                        SlopeIntensity *
                        Config.Tracks.ClimbingPenalty *
                        (DistanceStep / 100000.0f);
                }
                else if (Slope < -Config.Tracks.Thresholds.ClimbingSlopeThreshold)
                {
                    const float DescendingIntensity =
                        (-Slope - Config.Tracks.Thresholds.ClimbingSlopeThreshold) /
                        FMath::Max(
                            Config.Tracks.Thresholds.ClimbingSlopeThreshold,
                            KINDA_SMALL_NUMBER);

                    Penalty -=
                        DescendingIntensity *
                        Config.Tracks.DescendingSlopeBonus *
                        (DistanceStep / 100000.0f);
                }
            }

            if (!StartTangent.IsNearlyZero() &&
                !EndTangent.IsNearlyZero())
            {
                const float TangentDot =
                    FMath::Clamp(
                        FVector::DotProduct(StartTangent, EndTangent),
                        -1.0f,
                        1.0f);

                const float AngleChange =
                    FMath::Acos(TangentDot);

                if (AngleChange > KINDA_SMALL_NUMBER)
                {
                    // Radius = arc length / angle in radians.
                    const float Radius = DistanceStep / AngleChange;

                    if (Radius <
                        Config.Tracks.Thresholds.TightCurveRadiusThreshold)
                    {
                        const float CurveIntensity =
                            1.0f -
                            Radius /
                            FMath::Max(
                                Config.Tracks.Thresholds.TightCurveRadiusThreshold,
                                KINDA_SMALL_NUMBER);

                        Penalty +=
                            CurveIntensity *
                            Config.Tracks.TightCurvePenalty *
                            (DistanceStep / 100000.0f);
                    }
                }
            }
        }

        return Penalty;
    }
}


float FFactorioRailroadAStarFilter::CalculateFactorioPenalty(AFGBuildableRailroadTrack* Track,
    const AFGTrain* IgnoredTrain) const
{
    float Penalty = 0.0f;
    if (!IsValid(Track)) {
        return Penalty;
    }
    Penalty += CountStationPlatforms(Track->GetConnection(0), Config) + CountStationPlatforms(Track->GetConnection(1), Config);
    Penalty += CountVehiclesOnTrack(Track, Config,
        IgnoredTrain);
    Penalty += CalculateTrackGeometryPenalty(
        Track,
        Config);
    if (IsTrackInAnyTrainPath(Track, IgnoredTrain))
    {
        // Beispielwert
        Penalty += Config.Trains.PathReservationPenalty;
    }
    //Conn->GetTrack()->IsOccupied();
    // Block & Signal inspection
    // 14. Block has Path reservation: +25
    // 15. Block occupied by Train: +SegmentLength * 2.0f
    // 10. Train Long Waiting at Signal: +500
    // 11. Train Waiting at Path Signal: +200

    // Station penalties
    // 7. Train Station: +2000 (+0.25 per platform)
    // 4. Train Arriving at Station with Station as Destination: +2600
    // 5. Train Station with Train: +2500

    // Train State penalties (querying train occupying the block/station)
    // 1. Manual Train without Player: +6750
    // 2. Automatic Train without Schedule: +7000
    // 3. Derailed Train: +5000
    // 8. Manual Train with Player: +2000
    // 9. Train without Path: +1400

    return Penalty;
}

float FFactorioRailroadAStarFilter::GetTraversalCost(
    const FRailroadGraphAStarPathPoint& StartNodeRef,
    const FRailroadGraphAStarPathPoint& EndNodeRef,
    const AFGTrain* IgnoredTrain) const
{
    float OrigCost = BaseFilter.GetTraversalCost(StartNodeRef, EndNodeRef);

    if (!IsValid(StartNodeRef.TrackConnection) || !IsValid(EndNodeRef.TrackConnection))
    {
        UE_LOG(train_pathing, Warning, TEXT("TraversalCost TrackConnection not valid (Start: %p, End: %p)"), StartNodeRef.TrackConnection, EndNodeRef.TrackConnection);
        return 0.0f;
    }

    UFGRailroadTrackConnectionComponent* ConnB = EndNodeRef.TrackConnection;
    AFGBuildableRailroadTrack* Track = ConnB ? ConnB->GetTrack() : nullptr;

    if (!IsValid(Track))
    {
        UE_LOG(train_pathing, Warning, TEXT("TraversalCost Tracks invalid (Start: %p, End: %p)"), StartNodeRef.TrackConnection->GetTrack(), Track);
        return 0.0f;
    }

    float SegmentLength = Track ? Track->GetLength() : 1000.0f;

    // Base Cost (Rule 16: Length, slope, curvature adjustments)
    float BaseCost = SegmentLength;
    // Apply Factorio Penalty Table
    float Penalty = CalculateFactorioPenalty(Track, IgnoredTrain);
    float NewCost = BaseCost + Penalty * Config.Other.BasePenaltyScale;
    //UE_LOG(train_pathing, Verbose, TEXT("Traversal = %f <=> %f"), OrigCost, NewCost);
    return NewCost;
}

bool FFactorioRailroadAStarFilter::WantsPartialSolution() const
{
    return BaseFilter.WantsPartialSolution();
}

bool FFactorioRailroadAStarFilter::ShouldIncludeStartNodeInPath() const
{
    return BaseFilter.ShouldIncludeStartNodeInPath();
}


// ---------------------------------------------------------------------------------
// Debug HUD Configuration & Headers
// ---------------------------------------------------------------------------------
#define ENABLE_TRACK_DEBUG_HUD 1

#if ENABLE_TRACK_DEBUG_HUD


#endif


void FindPathSyncHook(auto& scope, AFGLocomotive* locomotive,
    AFGBuildableRailroadStation* station,
    FRailroadGraphAStarFilter filter)
{
    if (FTrainPathingConfigStruct::GetActiveConfig(locomotive).Debug.UseOriginalPathFinding) {
        scope.Override(scope(locomotive, station, filter));
        UE_LOG(train_pathing, Verbose, TEXT("Used Original Pathfinding"));
        return;
    }
    FRailroadPathFindingResult Result;
    Result.Locomotive = locomotive;
    Result.Result = ERailroadPathFindingResult::RPFR_Error;

    if (!IsValid(locomotive) || !IsValid(station))
    {
        UE_LOG(train_pathing, Verbose, TEXT("FindPathSyncHook: locomotive or station invalid (locomotive=%p, station=%p)"), locomotive, station);
        scope.Override(Result);
        return;
    }

    // Log locomotive track position candidates
    UFGRailroadTrackConnectionComponent* LocForward = locomotive->GetTrackPosition().GetForwardConnection();
    UFGRailroadTrackConnectionComponent* LocReverse = locomotive->GetTrackPosition().GetReverseConnection();
    UE_LOG(train_pathing, Verbose, TEXT("FindPathSyncHook: locomotive=%p, station=%p, LocForward=%p, LocReverse=%p"), locomotive, station, LocForward, LocReverse);

    // 1. Resolve Start & Goal Track Connections (robustly)
    UFGRailroadTrackConnectionComponent* StartConn = nullptr;
    {
        if (LocForward)
        {
            StartConn = LocForward;
        }
        else
        {
            StartConn = locomotive->GetTrackPosition().GetReverseConnection();
        }
    }

    UFGRailroadTrackConnectionComponent* GoalConn = nullptr;
    UFGRailroadTrackConnectionComponent* StationOutputConn = nullptr;
    UFGRailroadTrackConnectionComponent* StationForwardConn = nullptr;
    UFGRailroadTrackConnectionComponent* StationReverseConn = nullptr;

    if (station->GetStationOutputConnection())
    {
        StationOutputConn = station->GetStationOutputConnection()->GetRailroadConnectionReference();

        UE_LOG(
            train_pathing,
            Verbose,
            TEXT("StationOutputConn=%p"),
            StationOutputConn
        );

        if (StationOutputConn)
        {
            UFGRailroadTrackConnectionComponent* OppositeConn = StationOutputConn->GetOpposite();

            UE_LOG(
                train_pathing,
                Verbose,
                TEXT("StationOutputConn opposite=%p"),
                OppositeConn
            );

            /*
             * GetStationOutputConnection() references the platform-facing
             * endpoint. The train path, however, must end at the opposite
             * railroad connection of the station track.
             */
            if (OppositeConn)
            {
                GoalConn = OppositeConn;

                UE_LOG(
                    train_pathing,
                    Verbose,
                    TEXT("Using opposite of StationOutputConn as GoalConn: %p -> %p"),
                    StationOutputConn,
                    GoalConn
                );
            }
            else
            {
                GoalConn = StationOutputConn;

                UE_LOG(
                    train_pathing,
                    Warning,
                    TEXT("StationOutputConn has no opposite; using StationOutputConn directly as GoalConn=%p"),
                    GoalConn
                );
            }
        }
    }

    if (!IsValid(GoalConn))
    {
        StationForwardConn = station->GetTrackPosition().GetForwardConnection();
        StationReverseConn = station->GetTrackPosition().GetReverseConnection();

        UE_LOG(
            train_pathing,
            Verbose,
            TEXT("Station track-position candidates: Forward=%p Reverse=%p"),
            StationForwardConn,
            StationReverseConn
        );

        if (StationForwardConn)
        {
            GoalConn = StationForwardConn;
        }
        else
        {
            GoalConn = StationReverseConn;
        }

        UE_LOG(
            train_pathing,
            Verbose,
            TEXT("Using station track-position fallback as GoalConn=%p"),
            GoalConn
        );
    }

    if (!IsValid(StartConn) || !IsValid(GoalConn))
    {
        UE_LOG(train_pathing, Verbose, TEXT("locomotive or station connection invalid (StartConn=%p, GoalConn=%p)"), StartConn, GoalConn);
        Result.Result = ERailroadPathFindingResult::RPFR_Unreachable;
        scope.Override(Result);
        return;
    }

    Result.Goal = GoalConn;

    // 2. Setup A* Points
    // IMPORTANT: mark the start as an ignored-start so that A* does not treat it as matching the end incorrectly
    FRailroadGraphAStarPathPoint StartPoint(StartConn, true);
    FRailroadGraphAStarPathPoint GoalPoint(GoalConn);

    // 3. Run Custom A* with Factorio Cost Filter
    FRailroadGraphAStarHelper GraphHelper;
    FFactorioRailroadAStarFilter CustomFilter(filter, FTrainPathingConfigStruct::GetActiveConfig(locomotive));
    UE_LOG(train_pathing, Verbose, TEXT("BaseStationPenalty %f"), CustomFilter.Config.Platforms.StationBasePenalty);
    FGraphAStar<FRailroadGraphAStarHelper> AStarSolver(GraphHelper);

    TArray<FRailroadGraphAStarPathPoint> OutPathPoints;
    EGraphAStarResult AStarResult = AStarSolver.FindPath(StartPoint, GoalPoint, CustomFilter, OutPathPoints);

    // 3b. Validate A* result and path contents
    if (AStarResult != EGraphAStarResult::SearchSuccess)
    {
        UE_LOG(train_pathing, Verbose, TEXT("AStar did not succeed (result=%d)"), static_cast<int32>(AStarResult));
        Result.Result = ERailroadPathFindingResult::RPFR_Unreachable;
        scope.Override(Result);
        return;
    }

    if (OutPathPoints.Num() == 0)
    {
        UE_LOG(train_pathing, Warning, TEXT("AStar returned SearchSuccess but OutPathPoints is empty - aborting"));
        Result.Result = ERailroadPathFindingResult::RPFR_Unreachable;
        scope.Override(Result);
        return;
    }

    // Normalize & sanitize path points:
    // - Remove consecutive duplicates
    // - Ensure path begins with StartConn (the "next connection ahead" requirement)
    TArray<UFGRailroadTrackConnectionComponent*> NormalizedConns;
    NormalizedConns.Reserve(OutPathPoints.Num());

    for (const FRailroadGraphAStarPathPoint& Pt : OutPathPoints)
    {
        if (!IsValid(Pt.TrackConnection)) continue;
        UFGRailroadTrackConnectionComponent* Conn = Pt.TrackConnection;
        if (NormalizedConns.Num() == 0 || NormalizedConns.Last() != Conn)
        {
            NormalizedConns.Add(Conn);
        }
    }

    // Ensure start is present as first element
    if (filter.ShouldIncludeStartNodeInPath()) {
        if (NormalizedConns.Num() == 0 || NormalizedConns[0] != StartConn)
        {
            NormalizedConns.Insert(StartConn, 0);
        }
    }
    else if(NormalizedConns[0] == StartConn) {
        NormalizedConns.Remove(StartConn);
    }


    // Ensure goal is present as the final path point.
    bool bAppendedGoal = false;

    if (NormalizedConns.Num() == 0)
    {
        UE_LOG(
            train_pathing,
            Warning,
            TEXT("Normalized path is empty before adding GoalConn")
        );
    }

    if (!filter.WantsPartialSolution()) {
        if (NormalizedConns.Num() == 0 || NormalizedConns.Last() != GoalConn)
        {
            NormalizedConns.Add(GoalConn);
            bAppendedGoal = true;

            UE_LOG(
                train_pathing,
                Verbose,
                TEXT("Appended GoalConn=%p as final path point"),
                GoalConn
            );
        }
    }

    // 4. Construct the Railroad Path from normalized connections
    Result.Result = ERailroadPathFindingResult::RPFR_Success;
    Result.Path = MakeShared<FRailroadPath>();
    Result.Path->Station = station;

    for (UFGRailroadTrackConnectionComponent* Conn : NormalizedConns)
    {
        FRailroadPathPoint PathPt;
        PathPt.TrackConnection = Conn;
        Result.Path->PathPoints.Add(PathPt);
    }

    // --- Compute distances for each path point (distance to goal, last==0)
    {
        int32 NumPts = Result.Path->PathPoints.Num();
        if (NumPts > 0)
        {
            // last point distance zero
            Result.Path->PathPoints[NumPts - 1].Distance = 0.0f;

            for (int32 i = NumPts - 2; i >= 0; --i)
            {
                UFGRailroadTrackConnectionComponent* ConnA = Result.Path->PathPoints[i].TrackConnection.Get();
                UFGRailroadTrackConnectionComponent* ConnB = Result.Path->PathPoints[i + 1].TrackConnection.Get();
                float SegmentLen = 0.0f;

                if (ConnA && ConnB)
                {
                    AFGBuildableRailroadTrack* TrackA = ConnA->GetTrack();
                    AFGBuildableRailroadTrack* TrackB = ConnB->GetTrack();

                    if (TrackA && TrackB && TrackA == TrackB)
                    {
                        // Both connections on same track -> use track length as approximation
                        SegmentLen = TrackA->GetLength();
                    }
                    else
                    {
                        // Fallback to straight-line distance
                        SegmentLen = FVector::Dist(ConnA->GetComponentLocation(), ConnB->GetComponentLocation());
                    }
                }

                Result.Path->PathPoints[i].Distance = Result.Path->PathPoints[i + 1].Distance + SegmentLen;
            }
        }
    }

    if (Result.Path->PathPoints.Num() == 0)
    {
        UE_LOG(train_pathing, Warning, TEXT("Constructed FRailroadPath has zero PathPoints despite success - this should not happen"));
        Result.Result = ERailroadPathFindingResult::RPFR_Unreachable;
        scope.Override(Result);
        return;
    }

    UE_LOG(train_pathing, Verbose, TEXT("Path constructed with %d points (Start=%p, Goal=%p)"), Result.Path->PathPoints.Num(), StartConn, GoalConn);

    switch (Result.Result)
    {
    case ERailroadPathFindingResult::RPFR_Success:
        UE_LOG(train_pathing, Verbose, TEXT("Path found"));
        break;
    case ERailroadPathFindingResult::RPFR_Unreachable:
        UE_LOG(train_pathing, Verbose, TEXT("Path unreachable"));
        break;
    case ERailroadPathFindingResult::RPFR_Error:
        UE_LOG(train_pathing, Verbose, TEXT("Path error"));
        break;
    default:
        break;
    }
    // --- NEW: call original implementation and compare results before overriding
    {
        FRailroadPathFindingResult Orig = scope(locomotive, station, filter);
        UE_LOG(train_pathing, Verbose, TEXT("OriginalFindPath returned Result=%d Path=%p"), static_cast<int32>(Orig.Result), Orig.Path.Get());

        auto LogPathSummary = [](const FRailroadPathSharedPtr& P, const FString& Tag) {
            if (!P.IsValid()) {
                UE_LOG(train_pathing, Verbose, TEXT("[%s] Path == null"), *Tag);
                return;
            }
            UE_LOG(train_pathing, Verbose, TEXT("[%s] PathPoints=%d Station=%p"), *Tag, P->PathPoints.Num(), P->Station.Get());
            for (int32 i = 0; i < P->PathPoints.Num(); ++i)
            {
                const FRailroadPathPoint& PP = P->PathPoints[i];
                UFGRailroadTrackConnectionComponent* C = PP.TrackConnection.Get();
                FString OwnerName = C && C->GetOwner() ? C->GetOwner()->GetName() : TEXT("null");
                UE_LOG(train_pathing, Verbose, TEXT("  [%s] Pt[%d] Conn=%p Owner=%s Dist=%f"), *Tag, i, C, *OwnerName, PP.Distance);
            }
            };
        if (GoalConn)
        {
            AFGBuildableRailroadTrack* GoalTrack = GoalConn->GetTrack();
            UFGRailroadTrackConnectionComponent* GoalOpposite = GoalConn->GetOpposite();

            int32 GoalConnectionIndex = INDEX_NONE;

            if (GoalTrack)
            {
                for (int32 ConnectionIndex = 0; ConnectionIndex < 2; ++ConnectionIndex)
                {
                    if (GoalTrack->GetConnection(ConnectionIndex) == GoalConn)
                    {
                        GoalConnectionIndex = ConnectionIndex;
                        break;
                    }
                }
            }

            UE_LOG(
                train_pathing,
                Verbose,
                TEXT(
                    "Resolved GoalConn=%p Track=%p TrackLength=%f "
                    "ConnectionIndex=%d Opposite=%p"
                ),
                GoalConn,
                GoalTrack,
                GoalTrack ? GoalTrack->GetLength() : -1.0f,
                GoalConnectionIndex,
                GoalOpposite
            );
        }
        LogPathSummary(Orig.Path, TEXT("Original"));
        LogPathSummary(Result.Path, TEXT("Hooked"));

        // quick comparison summary
        int32 OrigNum = Orig.Path.IsValid() ? Orig.Path->PathPoints.Num() : 0;
        int32 NewNum = Result.Path.IsValid() ? Result.Path->PathPoints.Num() : 0;
        UE_LOG(train_pathing, Verbose, TEXT("PathCompare: OrigNum=%d NewNum=%d"), OrigNum, NewNum);

        if (Orig.Path.IsValid() && Result.Path.IsValid())
        {
            int32 Min = FMath::Min(OrigNum, NewNum);
            int32 DiffCount = 0;
            for (int32 i = 0; i < Min; ++i)
            {
                UFGRailroadTrackConnectionComponent* O = Orig.Path->PathPoints[i].TrackConnection.Get();
                UFGRailroadTrackConnectionComponent* N = Result.Path->PathPoints[i].TrackConnection.Get();
                if (O != N) DiffCount++;
            }
            UE_LOG(train_pathing, Verbose, TEXT("PathCompare: first %d entries differ in %d places"), Min, DiffCount);
        }

        // Erweiterte Details beim Path-Vergleich
        auto LogDetailedPoint = [](UFGRailroadTrackConnectionComponent* C, const FString& Tag, int32 Index)
        {
            if (!IsValid(C))
            {
                UE_LOG(train_pathing, Verbose, TEXT("  %s Pt[%d] Conn=null"), *Tag, Index);
                return;
            }

            AFGBuildableRailroadTrack* T = C->GetTrack();
            const TCHAR* OwnerName = C->GetOwner() ? *C->GetOwner()->GetName() : TEXT("unknown");
            float TrackLength = T ? T->GetLength() : -1.0f;

            FRailroadTrackPosition ConnPos = C->GetTrackPosition();
            float ForwardOffset = ConnPos.IsValid() ? ConnPos.GetForwardOffset() : -1.0f;
            float ReverseOffset = ConnPos.IsValid() ? ConnPos.GetReverseOffset() : -1.0f;

            // If we can, find which connection index on the track this is (0 or 1)
            int ConnIndex = INDEX_NONE;
            if (T)
            {
                for (int i = 0; i < 2; ++i)
                {
                    if (T->GetConnection(i) == C)
                    {
                        ConnIndex = i;
                        break;
                    }
                }
            }

            /*UE_LOG(train_pathing, Verbose, TEXT("  %s Pt[%d] Conn=%p Owner=%s Track=%p TrackLen=%f ConnIndex=%d ForwardOffset=%f ReverseOffset=%f"),
                *Tag, Index, C, OwnerName, T, TrackLength, ConnIndex, ForwardOffset, ReverseOffset);*/
        };

        if (Orig.Path.IsValid())
        {
            for (int i = 0; i < Orig.Path->PathPoints.Num(); ++i)
            {
                LogDetailedPoint(Orig.Path->PathPoints[i].TrackConnection.Get(), TEXT("Original"), i);
            }
        }
        if (Result.Path.IsValid())
        {
            for (int i = 0; i < Result.Path->PathPoints.Num(); ++i)
            {
                LogDetailedPoint(Result.Path->PathPoints[i].TrackConnection.Get(), TEXT("Hooked"), i);
            }
        }
    }
    scope.Override(Result);
}


void Ftrain_pathing_factoreworkModule::StartupModule()
{
    UE_LOG(train_pathing, Verbose, TEXT("Starting Train Pathing"));

    if (!WITH_EDITOR)
    {
        UE_LOG(train_pathing, Verbose, TEXT("Hooking Path Finder"));

        SUBSCRIBE_METHOD(
            FRailroadNavigation::FindPathSync,
            [](auto& scope,
               AFGLocomotive* locomotive,
               AFGBuildableRailroadStation* station,
               FRailroadGraphAStarFilter filter)
            {
                FindPathSyncHook(
                    scope,
                    locomotive,
                    station,
                    filter);
            });

        TrainPathingDebug::Startup();

        UE_LOG(
            train_pathing,
            Verbose,
            TEXT("Hooked Path Finder"));
    }
    else
    {
        UE_LOG(train_pathing, Verbose, TEXT("Not Hooked!"));
    }
}

void Ftrain_pathing_factoreworkModule::ShutdownModule()
{
    TrainPathingDebug::Shutdown();
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(Ftrain_pathing_factoreworkModule, train_pathing_factorework)

