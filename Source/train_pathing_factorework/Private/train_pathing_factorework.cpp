// Copyright Epic Games, Inc. All Rights Reserved.

#include "train_pathing_factorework.h"
#include "BP_TrainPathingFactoreworkConfigStruct.h"

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

static float CountVehiclesOnTrack(
    AFGBuildableRailroadTrack* Track,
    const FBP_TrainPathingFactoreworkConfigStruct& Config)
{
    float Counts = 0.0f;

    if (!IsValid(Track))
    {
        return Counts;
    }

    for (const TObjectPtr<AFGRailroadVehicle>& Vehicle :
        Track->GetVehicles())
    {
        AFGRailroadVehicle* VehicleActor = Vehicle.Get();

        if (!IsValid(VehicleActor))
        {
            continue;
        }
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
    return Counts;
}

static float CountStationPlatforms(
    UFGRailroadTrackConnectionComponent* RailroadConnection,
    const FBP_TrainPathingFactoreworkConfigStruct& Config)
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

struct FFactorioRailroadAStarFilter :
    public FRailroadGraphAStarFilter
{
    const FRailroadGraphAStarFilter& BaseFilter;
    FBP_TrainPathingFactoreworkConfigStruct Config;

    FFactorioRailroadAStarFilter(
        const FRailroadGraphAStarFilter& InBase,
        const FBP_TrainPathingFactoreworkConfigStruct& InConfig)
        : BaseFilter(InBase)
        , Config(InConfig)
    {
    }

    float GetHeuristicScale() const {
        return BaseFilter.GetHeuristicScale();
    }

    float GetHeuristicCost(const FRailroadGraphAStarPathPoint& StartNodeRef, const FRailroadGraphAStarPathPoint& EndNodeRef) const
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

    bool IsTraversalAllowed(const FRailroadGraphAStarPathPoint& NodeA, const FRailroadGraphAStarPathPoint& NodeB) const
    {
        bool bAllowed = BaseFilter.IsTraversalAllowed(NodeA, NodeB);
        //UE_LOG(train_pathing, Verbose, TEXT("Orig_IsTraversal = %d"), bAllowed ? 1 : 0);

        if (!IsValid(NodeA.TrackConnection) || !IsValid(NodeB.TrackConnection))
        {
            return false;
        }

        return bAllowed;
    }

    float CalculateFactorioPenalty(AFGBuildableRailroadTrack* Track) const
    {
        float Penalty = 0.0f;
        if (!IsValid(Track)) {
            return Penalty;
        }
        Penalty += CountStationPlatforms(Track->GetConnection(0), Config) + CountStationPlatforms(Track->GetConnection(1), Config);
        Penalty += CountVehiclesOnTrack(Track, Config);
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

    float GetTraversalCost(const FRailroadGraphAStarPathPoint& StartNodeRef, const FRailroadGraphAStarPathPoint& EndNodeRef) const
    {
        float OrigCost = BaseFilter.GetTraversalCost(StartNodeRef, EndNodeRef);

        if (!IsValid(StartNodeRef.TrackConnection) || !IsValid(EndNodeRef.TrackConnection))
        {
            UE_LOG(train_pathing, Warning, TEXT("TraversalCost TrackConnection not valid (Start: %p, End: %p)"), StartNodeRef.TrackConnection, EndNodeRef.TrackConnection);
            return 0.0f;
        }

        UFGRailroadTrackConnectionComponent* ConnB = EndNodeRef.TrackConnection;
        AFGBuildableRailroadTrack* Track = ConnB ? ConnB->GetTrack() : nullptr;

        if (!IsValid(Track) || Track != StartNodeRef.TrackConnection->GetTrack())
        {
            UE_LOG(train_pathing, Warning, TEXT("TraversalCost Tracks differ (Start: %p, End: %p)"), StartNodeRef.TrackConnection->GetTrack(), Track);
            return 0.0f;
		}

        float SegmentLength = Track ? Track->GetLength() : 1000.0f;

        // Base Cost (Rule 16: Length, slope, curvature adjustments)
        float BaseCost = SegmentLength;

        // Apply Factorio Penalty Table
        float Penalty = CalculateFactorioPenalty(Track);
        float NewCost = BaseCost + Penalty;
        //UE_LOG(train_pathing, Verbose, TEXT("Traversal = %f <=> %f"), OrigCost, NewCost);
        return NewCost;
    }

    bool WantsPartialSolution() const { return BaseFilter.WantsPartialSolution(); }
    bool ShouldIncludeStartNodeInPath() const { return BaseFilter.ShouldIncludeStartNodeInPath(); }
};


// ---------------------------------------------------------------------------------
// Debug HUD Configuration & Headers
// ---------------------------------------------------------------------------------
#define ENABLE_TRACK_DEBUG_HUD 1

#if ENABLE_TRACK_DEBUG_HUD
#include "FGPlayerController.h"
#include "Camera/PlayerCameraManager.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "CollisionQueryParams.h"
#include "FGTrain.h"
#include "FGRailroadVehicle.h"
#include "FGLocomotive.h"
#include "FGCharacterPlayer.h"
#include "EngineUtils.h"

// State variables to cache the inspected track
static FString GInspectedTrackText = TEXT("Looking at Track: [None]");
static float GTrackTraceTimer = 0.0f;
static constexpr float TRACE_INTERVAL = 0.2f; // Run raycast every 0.2s
static AFGBuildableRailroadTrack* lastTrack = nullptr;
static TArray<TWeakObjectPtr<AActor>> GConnectionMarkerActors;
static TWeakObjectPtr<AFGBuildableRailroadTrack> GMarkerTrack;
static TWeakObjectPtr<AFGTrain> GHighlightedTrain;
static FRailroadPathSharedPtr GHighlightedPath;
static TArray<TWeakObjectPtr<AFGBuildableRailroadTrack>> GHighlightedPathTracks;

static void StopHighlightedTrainPath()
{
    for (TWeakObjectPtr<AFGBuildableRailroadTrack>& Track :
        GHighlightedPathTracks)
    {
        if (Track.IsValid() && Track.Get() != lastTrack)
        {
            Track->StopBlockVisualization();
        }
    }

    GHighlightedPathTracks.Reset();
    GHighlightedPath.Reset();
    GHighlightedTrain.Reset();
}

static bool ContainsTrack(
    const TArray<TWeakObjectPtr<AFGBuildableRailroadTrack>>& Tracks,
    AFGBuildableRailroadTrack* Track)
{
    for (const TWeakObjectPtr<AFGBuildableRailroadTrack>& ExistingTrack : Tracks)
    {
        if (ExistingTrack.Get() == Track)
        {
            return true;
        }
    }

    return false;
}

static AFGTrain* GetPlayerTrain(AFGPlayerController* PlayerController)
{
    if (!IsValid(PlayerController))
    {
        return nullptr;
    }

    APawn* PlayerPawn = PlayerController->GetPawn();

    // Fall 1: Der Spieler besitzt direkt ein Railroad Vehicle.
    AFGRailroadVehicle* RailroadVehicle =
        Cast<AFGRailroadVehicle>(PlayerPawn);

    if (IsValid(RailroadVehicle) && IsValid(RailroadVehicle->GetTrain()))
    {
        return RailroadVehicle->GetTrain();
    }

    // Fall 2: Der Spieler ist weiterhin als Character-Pawn vorhanden
    // und sitzt als Fahrer in einer Lokomotive.
    AFGCharacterPlayer* Character =
        Cast<AFGCharacterPlayer>(PlayerPawn);

    if (!IsValid(Character) || !IsValid(PlayerController->GetWorld()))
    {
        return nullptr;
    }

    for (TActorIterator<AFGLocomotive> Iterator(PlayerController->GetWorld());
        Iterator;
        ++Iterator)
    {
        AFGLocomotive* Locomotive = *Iterator;

        if (Locomotive &&
            Locomotive->GetDriver() == Character &&
            Locomotive->GetTrain())
        {
            return Locomotive->GetTrain();
        }
    }

    return nullptr;
}

static void UpdatePlayerTrainPathVisualization(
    AFGPlayerController* PlayerController)
{
    AFGTrain* EnteredTrain = GetPlayerTrain(PlayerController);

    /*
     * Keep displaying the last train path after the player leaves the train.
     * When the player enters another train, EnteredTrain replaces the cached
     * train and the old path is removed below.
     */
    AFGTrain* CurrentTrain = EnteredTrain;

    if (!IsValid(CurrentTrain))
    {
        CurrentTrain = GHighlightedTrain.Get();
    }

    if (!IsValid(CurrentTrain))
    {
        if (GHighlightedPathTracks.Num() > 0)
        {
            StopHighlightedTrainPath();
        }

        return;
    }

    const FRailroadPathSharedPtr CurrentPath =
        CurrentTrain->mAtcData.Path;

    if (!CurrentPath.IsValid() ||
        CurrentPath->PathPoints.Num() == 0)
    {
        StopHighlightedTrainPath();
        return;
    }

    const int32 PathPointCount = CurrentPath->PathPoints.Num();

    int32 FirstPathPoint = CurrentTrain->mAtcData.CurrentPathSegment;

    /*
     * CurrentPathSegment is INDEX_NONE while the ATC state is being
     * initialized. In that case, display the complete path temporarily.
     */
    if (FirstPathPoint == INDEX_NONE)
    {
        FirstPathPoint = 0;
    }

    FirstPathPoint = FMath::Clamp(
        FirstPathPoint,
        0,
        PathPointCount - 1
    );

    TArray<TWeakObjectPtr<AFGBuildableRailroadTrack>>
        DesiredTracks;

    for (int32 PathPointIndex = FirstPathPoint;
        PathPointIndex < PathPointCount;
        ++PathPointIndex)
    {
        UFGRailroadTrackConnectionComponent* Connection =
            CurrentPath->PathPoints[PathPointIndex].TrackConnection.Get();

        if (!IsValid(Connection))
        {
            UE_LOG(
                train_pathing,
                Warning,
                TEXT(
                    "Train path contains invalid connection at "
                    "PathPoint[%d]"
                ),
                PathPointIndex
            );

            continue;
        }

        AFGBuildableRailroadTrack* Track =
            Connection->GetTrack();

        if (!IsValid(Track))
        {
            UE_LOG(
                train_pathing,
                Warning,
                TEXT(
                    "Train path connection has no track: "
                    "PathPoint[%d] Connection=%p"
                ),
                PathPointIndex,
                Connection
            );

            continue;
        }

        if (!ContainsTrack(DesiredTracks, Track))
        {
            DesiredTracks.Add(Track);
        }
    }

    /*
     * Remove tracks that belonged to the previous path or have already
     * been passed by the train.
     */
    for (TWeakObjectPtr<AFGBuildableRailroadTrack>& OldTrack :
        GHighlightedPathTracks)
    {
        if (!OldTrack.IsValid())
        {
            continue;
        }

        if (!ContainsTrack(DesiredTracks, OldTrack.Get()) &&
            OldTrack.Get() != lastTrack)
        {
            OldTrack->StopBlockVisualization();
        }
    }

    /*
     * Add newly required tracks and restore visualization if the game
     * disabled it after a track or signal was built.
     *
     * DesiredTracks is rebuilt from CurrentTrain->mAtcData.Path on every
     * update. Therefore passed tracks and tracks from an old path are not
     * re-enabled.
     */
    for (TWeakObjectPtr<AFGBuildableRailroadTrack>& Track :
        DesiredTracks)
    {
        if (!Track.IsValid())
        {
            continue;
        }

        /*
         * A track can still be part of GHighlightedPathTracks while the
         * game has removed its block visualization. Restore it in that case.
         */
        if (!Track->IsBlockVisualizationActive())
        {
            Track->ShowBlockVisualization();
        }
    }

    const bool bTrainChanged =
        GHighlightedTrain.Get() != CurrentTrain;

    const bool bPathChanged =
        GHighlightedPath.Get() != CurrentPath.Get();

    const bool bProgressChanged =
        GHighlightedPathTracks.Num() != DesiredTracks.Num();

    GHighlightedTrain = CurrentTrain;
    GHighlightedPath = CurrentPath;
    GHighlightedPathTracks = MoveTemp(DesiredTracks);
}

static void ClearTrackConnectionMarkers()
{
    for (TWeakObjectPtr<AActor>& MarkerActor : GConnectionMarkerActors)
    {
        if (MarkerActor.IsValid())
        {
            MarkerActor->Destroy();
        }
    }

    GConnectionMarkerActors.Reset();
    GMarkerTrack.Reset();
}

static FVector GetTrackConnectionMarkerLocation(
    UFGRailroadTrackConnectionComponent* Connection)
{
    if (!IsValid(Connection))
    {
        return FVector::ZeroVector;
    }

    // Unreal-Einheiten sind Zentimeter.
    // 100 cm über der Connection sorgen dafür, dass der Marker nicht
    // zwischen beziehungsweise unter den Gleisen verschwindet.
    return Connection->GetComponentLocation() + FVector(0.0f, 0.0f, 150.0f);
}

static AActor* CreateTrackConnectionMarker(
    UWorld* World,
    UFGRailroadTrackConnectionComponent* Connection,
    const FLinearColor& Color)
{
    if (!IsValid(World) || !IsValid(Connection))
    {
        return nullptr;
    }

    FActorSpawnParameters SpawnParameters;
    SpawnParameters.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    AActor* MarkerActor = World->SpawnActor<AActor>(
        AActor::StaticClass(),
        GetTrackConnectionMarkerLocation(Connection),
        FRotator::ZeroRotator,
        SpawnParameters
    );

    if (!IsValid(MarkerActor))
    {
        return nullptr;
    }

    UStaticMeshComponent* MarkerMesh =
        NewObject<UStaticMeshComponent>(MarkerActor);

    if (!IsValid(MarkerMesh))
    {
        MarkerActor->Destroy();
        return nullptr;
    }

    MarkerActor->SetRootComponent(MarkerMesh);
    MarkerMesh->RegisterComponent();

    UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(
        nullptr,
        TEXT("/Engine/BasicShapes/Sphere.Sphere")
    );

    if (!IsValid(SphereMesh))
    {
        UE_LOG(
            train_pathing,
            Warning,
            TEXT("Could not load sphere mesh for track connection marker")
        );

        MarkerActor->Destroy();
        return nullptr;
    }

    MarkerMesh->SetStaticMesh(SphereMesh);
    MarkerMesh->SetMobility(EComponentMobility::Movable);
    MarkerMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    MarkerMesh->SetGenerateOverlapEvents(false);
    MarkerMesh->SetCastShadow(false);

    // Die Standard-Sphere hat ungefähr 100 cm Durchmesser.
    MarkerActor->SetActorScale3D(FVector(0.45f));
    MarkerActor->SetActorLocation(GetTrackConnectionMarkerLocation(Connection));

    UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(
        nullptr,
        TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial")
    );

    if (BaseMaterial)
    {
        UMaterialInstanceDynamic* DynamicMaterial =
            UMaterialInstanceDynamic::Create(BaseMaterial, MarkerActor);

        if (DynamicMaterial)
        {
            DynamicMaterial->SetVectorParameterValue(
                TEXT("Color"),
                Color
            );

            MarkerMesh->SetMaterial(0, DynamicMaterial);
        }
    }

    return MarkerActor;
}

static void UpdateTrackConnectionMarkers(
    UWorld* World,
    AFGBuildableRailroadTrack* Track)
{
    if (!IsValid(World) || !IsValid(Track))
    {
        ClearTrackConnectionMarkers();
        return;
    }

    if (GMarkerTrack.Get() == Track &&
        GConnectionMarkerActors.Num() == 2)
    {
        for (int32 ConnectionIndex = 0; ConnectionIndex < 2; ++ConnectionIndex)
        {
            UFGRailroadTrackConnectionComponent* Connection =
                Track->GetConnection(ConnectionIndex);

            AActor* MarkerActor =
                GConnectionMarkerActors[ConnectionIndex].Get();

            if (Connection && MarkerActor)
            {
                MarkerActor->SetActorLocation(
                    GetTrackConnectionMarkerLocation(Connection)
                );
            }
        }

        return;
    }

    ClearTrackConnectionMarkers();

    const FLinearColor IndexZeroColor = FLinearColor::Red;
    const FLinearColor IndexOneColor = FLinearColor::Green;

    const FLinearColor Colors[2] =
    {
        IndexZeroColor,
        IndexOneColor
    };

    for (int32 ConnectionIndex = 0; ConnectionIndex < 2; ++ConnectionIndex)
    {
        UFGRailroadTrackConnectionComponent* Connection =
            Track->GetConnection(ConnectionIndex);

        if (!IsValid(Connection))
        {
            UE_LOG(
                train_pathing,
                Warning,
                TEXT(
                    "Track=%p has no connection at index %d"
                ),
                Track,
                ConnectionIndex
            );

            continue;
        }

        AActor* MarkerActor = CreateTrackConnectionMarker(
            World,
            Connection,
            Colors[ConnectionIndex]
        );

        if (MarkerActor)
        {
            GConnectionMarkerActors.Add(MarkerActor);
        }
    }

    GMarkerTrack = Track;
}

// 1. Raycast helper to find the track under the crosshair
static AFGBuildableRailroadTrack* GetLookedAtRailTrack(UWorld* World, APlayerController* PC)
{
    if (!IsValid(World) || !IsValid(PC) || !IsValid(PC->PlayerCameraManager)) return nullptr;

    FVector CamLoc = PC->PlayerCameraManager->GetCameraLocation();
    FVector CamForward = PC->PlayerCameraManager->GetCameraRotation().Vector();
    FVector TraceEnd = CamLoc + (CamForward * 5000.0f); // 50m reach

    FHitResult Hit;
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(PC->GetPawn());

    if (World->LineTraceSingleByChannel(Hit, CamLoc, TraceEnd, ECC_Visibility, Params))
    {
        return Cast<AFGBuildableRailroadTrack>(Hit.GetActor());
    }

    return nullptr;
}

// 2. Periodic trace update called from PlayerTick
static void UpdateInspectedTrackData(
    AFGPlayerController* FGPC,
    float DeltaSeconds)
{
    if (!IsValid(FGPC))
    {
        return;
    }

    const FBP_TrainPathingFactoreworkConfigStruct Config =
        FBP_TrainPathingFactoreworkConfigStruct::GetActiveConfig(FGPC);

    if (!Config.Debug.EnableDebugHud)
    {
        ClearTrackConnectionMarkers();
        GInspectedTrackText = TEXT("Track debug HUD disabled");
        return;
    }

    GTrackTraceTimer += DeltaSeconds;
    if (GTrackTraceTimer < TRACE_INTERVAL)
    {
        return;
    }
    GTrackTraceTimer = 0.0f;
    AFGBuildableRailroadTrack* nTrack = GetLookedAtRailTrack(FGPC->GetWorld(), FGPC);
    if (IsValid(nTrack))
    {
        UpdateTrackConnectionMarkers(FGPC->GetWorld(), nTrack);

        UFGRailroadTrackConnectionComponent* BeginningConnection =
            nTrack->GetConnection(0);

        UFGRailroadTrackConnectionComponent* EndConnection =
            nTrack->GetConnection(1);

        if (!IsValid(BeginningConnection) || !IsValid(EndConnection))
        {
            GInspectedTrackText = FString::Printf(
                TEXT(
                    "Looking at Track: %s | Ptr: %p\n"
                    "Length: %f\n"
                    "Track connections are not initialized yet."
                ),
                *nTrack->GetName(),
                nTrack,
                nTrack->GetLength()
            );

            return;
        }

        FRailroadGraphAStarFilter origFilter;
        FFactorioRailroadAStarFilter Filter(origFilter, Config);

        GInspectedTrackText = FString::Printf(
            TEXT(
                "Looking at Track: %s | Ptr: %p\n"
                "Length: %f\n"
                "Beginning [0]: %p\n"
                "End [1]: %p\n"
                "Heuristics: %f\n"
                "Traversal: %f\n"
                "IsTraversalAllowed: %d\n"
                "Rev Heuristics: %f\n"
                "Rev Traversal: %f\n"
                "Rev IsTraversalAllowed: %d\n"
            ),
            *nTrack->GetName(),
            nTrack,
            nTrack->GetLength(),
            BeginningConnection,
            EndConnection,
            Filter.GetHeuristicCost(
                FRailroadGraphAStarPathPoint(BeginningConnection),
                FRailroadGraphAStarPathPoint(EndConnection)),
            Filter.GetTraversalCost(
                FRailroadGraphAStarPathPoint(BeginningConnection),
                FRailroadGraphAStarPathPoint(EndConnection)),
            Filter.IsTraversalAllowed(
                FRailroadGraphAStarPathPoint(BeginningConnection),
                FRailroadGraphAStarPathPoint(EndConnection)) ? 1 : 0,
            Filter.GetHeuristicCost(
                FRailroadGraphAStarPathPoint(EndConnection),
                FRailroadGraphAStarPathPoint(BeginningConnection)),
            Filter.GetTraversalCost(
                FRailroadGraphAStarPathPoint(EndConnection),
                FRailroadGraphAStarPathPoint(BeginningConnection)),
            Filter.IsTraversalAllowed(
                FRailroadGraphAStarPathPoint(EndConnection),
                FRailroadGraphAStarPathPoint(BeginningConnection)) ? 1 : 0
        );

        // Bestehende Blockvisualisierung beibehalten.
        if (lastTrack != nTrack)
        {
            if (lastTrack &&
                !ContainsTrack(GHighlightedPathTracks, lastTrack))
            {
                lastTrack->StopBlockVisualization();
            }

            nTrack->ShowBlockVisualization();
            lastTrack = nTrack;
        }
    }
    else
    {
        ClearTrackConnectionMarkers();

        if (lastTrack &&
            !ContainsTrack(GHighlightedPathTracks, lastTrack))
        {
            lastTrack->StopBlockVisualization();
            lastTrack = nullptr;
        }
        else if (lastTrack)
        {
            /*
             * The track is still highlighted by the train path.
             * Only remove the connection markers, not the track highlight.
             */
            lastTrack = nullptr;
        }

        GInspectedTrackText = TEXT("Looking at Track: [None]");
    }
}
static void DrawTrackHUD_Canvas(AHUD* HUD)
{
    if (!IsValid(HUD))
    {
        return;
    }

    const FBP_TrainPathingFactoreworkConfigStruct Config =
        FBP_TrainPathingFactoreworkConfigStruct::GetActiveConfig(HUD);

    if (!Config.Debug.EnableDebugHud)
    {
        return;
    }


    UFont* Font = GEngine ? GEngine->GetSmallFont() : nullptr;
    if (!Font) return;

    const float ScreenX = 50.0f;
    const float ScreenY = 150.0f;

    // 1. Schatten zeichnen
    HUD->DrawText(
        GInspectedTrackText,
        FLinearColor::Black,
        ScreenX + 1.0f,
        ScreenY + 1.0f,
        Font,
        1.0f,   // Scale
        false   // bDontScale
    );

    // 2. Cyan Text zeichnen
    HUD->DrawText(
        GInspectedTrackText,
        FLinearColor(0.0f, 1.0f, 1.0f, 1.0f),
        ScreenX,
        ScreenY,
        Font,
        1.0f,
        false
    );
}

#endif


void FindPathSyncHook(auto& scope, AFGLocomotive* locomotive,
    AFGBuildableRailroadStation* station,
    FRailroadGraphAStarFilter filter)
{
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
    FFactorioRailroadAStarFilter CustomFilter(filter, FBP_TrainPathingFactoreworkConfigStruct::GetActiveConfig(locomotive));
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
        SUBSCRIBE_METHOD(FRailroadNavigation::FindPathSync, [](auto& scope,
            AFGLocomotive* locomotive,
            AFGBuildableRailroadStation* station,
            FRailroadGraphAStarFilter filter)
            {
                FindPathSyncHook(scope, locomotive, station, filter);
            });
        UE_LOG(train_pathing, Verbose, TEXT("Hooked Path Finder"));
#if ENABLE_TRACK_DEBUG_HUD
        // 1. Tick hook for raycasting
        AFGPlayerController* PCDefault = GetMutableDefault<AFGPlayerController>();
        SUBSCRIBE_METHOD_VIRTUAL(AFGPlayerController::PlayerTick, PCDefault, [](auto& scope, AFGPlayerController* self, float DeltaSeconds)
            {
                scope(self, DeltaSeconds);
                if (self &&
                    self->IsLocalController() &&
                    FBP_TrainPathingFactoreworkConfigStruct::GetActiveConfig(self).Debug.EnableDebugHud)
                {
                    UpdateInspectedTrackData(self, DeltaSeconds);
                    UpdatePlayerTrainPathVisualization(self);
                }
            });

        // 2. HUD Canvas hook for continuous on-screen text
        AHUD* HUDDefault = GetMutableDefault<AHUD>();
        SUBSCRIBE_METHOD_VIRTUAL(AHUD::DrawHUD, HUDDefault, [](auto& scope, AHUD* self)
            {
                scope(self);
                if (self)
                {
                    DrawTrackHUD_Canvas(self);
                }
            });
        UE_LOG(train_pathing, Verbose, TEXT("Hooked PlayerTick for Track Debug HUD"));
#endif
    }
    else
    {
        UE_LOG(train_pathing, Verbose, TEXT("Not Hooked!"));
    }
}

void Ftrain_pathing_factoreworkModule::ShutdownModule()
{
#if ENABLE_TRACK_DEBUG_HUD
    StopHighlightedTrainPath();
    ClearTrackConnectionMarkers();

    if (lastTrack)
    {
        lastTrack->StopBlockVisualization();
    }
    lastTrack = nullptr;
    GInspectedTrackText = TEXT("Looking at Track: [None]");
    GTrackTraceTimer = 0.0f;
    GConnectionMarkerActors.Empty();
    GMarkerTrack = nullptr;
    GHighlightedTrain = nullptr;
    GHighlightedPath = nullptr;
    GHighlightedPathTracks.Empty();
#endif
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(Ftrain_pathing_factoreworkModule, train_pathing_factorework)
