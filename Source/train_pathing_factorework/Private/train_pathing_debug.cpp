// Copyright Epic Games, Inc. All Rights Reserved.

#include "train_pathing_debug.h"
#include "TrainPathingConfigStruct.h"
#include "train_pathing_factorework.h"
#include "Patching/NativeHookManager.h"

DEFINE_LOG_CATEGORY(train_pathing_debug);

#define ENABLE_TRACK_DEBUG_HUD 1

#if ENABLE_TRACK_DEBUG_HUD

#include "FGCharacterPlayer.h"
#include "FGPlayerController.h"
#include "FGTrain.h"
#include "FGRailroadVehicle.h"
#include "FGLocomotive.h"
#include "FGHUD.h"

#include "Buildables/FGBuildableRailroadTrack.h"
#include "FGRailroadTrackConnectionComponent.h"

#include "Camera/PlayerCameraManager.h"
#include "CollisionQueryParams.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Materials/MaterialInstanceDynamic.h"

namespace
{

    struct Trackdir
    {
        TWeakObjectPtr<AFGBuildableRailroadTrack> Track;
        bool reversed;
        Trackdir(
            AFGBuildableRailroadTrack* InTrack,
            bool bInReversed)
            : Track(InTrack)
            , reversed(bInReversed)
        {
        }
    };

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
    static TArray<Trackdir> GManuallySelectedTracks;

    static float GManualSelectionAccumulatedPenalty = 0.0f;
    static float GManualSelectionAccumulatedPenaltyTrain = 0.0f;
    static float GManualSelectionMinimumPenalty = 0.0f;
    static float GManualSelectionMaximumPenalty = 0.0f;
    static TWeakObjectPtr<AFGTrain> GLastEnteredManualTrain;

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

    static bool ContainsTrack(
        const TArray<Trackdir>& Tracks,
        AFGBuildableRailroadTrack* Track)
    {
        for (const Trackdir& ExistingTrack : Tracks)
        {
            if (ExistingTrack.Track.Get() == Track)
            {
                return true;
            }
        }

        return false;
    }

    static bool IsManuallySelectedTrack(
        AFGBuildableRailroadTrack* Track)
    {
        if (!IsValid(Track))
        {
            return false;
        }

        return ContainsTrack(GManuallySelectedTracks, Track);
    }

    static bool IsTrainPathTrack(
        AFGBuildableRailroadTrack* Track)
    {
        if (!IsValid(Track))
        {
            return false;
        }

        return ContainsTrack(GHighlightedPathTracks, Track);
    }

    static void StopTrackVisualizationIfUnused(
        AFGBuildableRailroadTrack* Track)
    {
        if (!IsValid(Track))
        {
            return;
        }

        if (Track == lastTrack ||
            IsManuallySelectedTrack(Track) ||
            IsTrainPathTrack(Track))
        {
            return;
        }

        Track->StopBlockVisualization();
    }

    static void ClearManualTrackSelection()
    {
        for (Trackdir& Trackd :
            GManuallySelectedTracks)
        {
            if (Trackd.Track.IsValid())
            {
                AFGBuildableRailroadTrack* TrackActor =
                    Trackd.Track.Get();

                /*
                 * Der Track kann gleichzeitig Teil des berechneten Pfades
                 * oder der Viewport-Auswahl sein.
                 */
                if (TrackActor != lastTrack &&
                    !IsTrainPathTrack(TrackActor))
                {
                    TrackActor->StopBlockVisualization();
                }
            }
        }

        GManuallySelectedTracks.Reset();
        GLastEnteredManualTrain.Reset();

        GManualSelectionAccumulatedPenalty = 0.0f;
        GManualSelectionAccumulatedPenaltyTrain = 0.0f;
        GManualSelectionMinimumPenalty = 0.0f;
        GManualSelectionMaximumPenalty = 0.0f;
    }


    static void StopHighlightedTrainPath()
    {
        for (TWeakObjectPtr<AFGBuildableRailroadTrack>& Track :
            GHighlightedPathTracks)
        {
            if (!Track.IsValid())
            {
                continue;
            }

            AFGBuildableRailroadTrack* TrackActor = Track.Get();

            if (TrackActor != lastTrack &&
                !IsManuallySelectedTrack(TrackActor))
            {
                TrackActor->StopBlockVisualization();
            }
        }

        GHighlightedPathTracks.Reset();
        GHighlightedPath.Reset();
        GHighlightedTrain.Reset();
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

    static void ClearTrackDebugVisualization()
    {
        StopHighlightedTrainPath();
        ClearTrackConnectionMarkers();
        ClearManualTrackSelection();

        if (lastTrack)
        {
            lastTrack->StopBlockVisualization();
            lastTrack = nullptr;
        }

        GInspectedTrackText.Empty();
        GTrackTraceTimer = 0.0f;
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
        if (!IsValid(PlayerController))
        {
            return;
        }

        const FTrainPathingConfigStruct Config =
            FTrainPathingConfigStruct::GetActiveConfig(PlayerController);

        if (!Config.Debug.EnableDebugHud)
        {
            ClearTrackDebugVisualization();
            return;
        }

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
                    train_pathing_debug,
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
                    train_pathing_debug,
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
                OldTrack.Get() != lastTrack &&
                !IsManuallySelectedTrack(OldTrack.Get()))
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
                train_pathing_debug,
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
                    train_pathing_debug,
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

        const FTrainPathingConfigStruct Config =
            FTrainPathingConfigStruct::GetActiveConfig(FGPC);

        if (!Config.Debug.EnableDebugHud)
        {
            ClearTrackDebugVisualization();
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

            /*
             * Ein A*-Kantenpaar (StartNodeRef, EndNodeRef) beschreibt das
             * Durchqueren GENAU EINES Tracks: EndNodeRef liegt auf diesem
             * Track (dem hier betrachteten "nTrack"), während StartNodeRef
             * auf dem VORHERIGEN, physisch damit verbundenen Track liegt.
             * BeginningConnection/EndConnection sind beide Enden von nTrack
             * selbst und dürfen daher NICHT gleichzeitig als Start und Ende
             * eines Kantenpaars verwendet werden - das würde bedeuten, der
             * "vorherige Track" wäre identisch mit nTrack selbst.
             *
             * Stattdessen wird die tatsächliche Eintritts-Connection über
             * die reale Verbindung zum vorherigen Track ermittelt:
             * EntryConnection->GetConnection() liefert die gegenüberliegende
             * Connection auf dem vorherigen Track.
             *
             * Da ein frei betrachteter Track keine Fahrtrichtung hat, werden
             * beide möglichen Durchquerungsrichtungen ausgewertet.
             */
            UFGRailroadTrackConnectionComponent* PrevTrackConnFromBeginning =
                BeginningConnection->GetConnection();

            UFGRailroadTrackConnectionComponent* PrevTrackConnFromEnd =
                EndConnection->GetConnection();

            const bool bHasForwardPrev = IsValid(PrevTrackConnFromBeginning);
            const bool bHasReversePrev = IsValid(PrevTrackConnFromEnd);

            const float ForwardHeuristic = bHasForwardPrev
                ? Filter.GetHeuristicCost(
                    FRailroadGraphAStarPathPoint(PrevTrackConnFromBeginning),
                    FRailroadGraphAStarPathPoint(EndConnection))
                : 0.0f;

            const float ForwardTraversal = bHasForwardPrev
                ? Filter.GetTraversalCost(
                    FRailroadGraphAStarPathPoint(PrevTrackConnFromBeginning),
                    FRailroadGraphAStarPathPoint(EndConnection))
                : 0.0f;

            const bool bForwardAllowed = bHasForwardPrev
                ? Filter.IsTraversalAllowed(
                    FRailroadGraphAStarPathPoint(PrevTrackConnFromBeginning),
                    FRailroadGraphAStarPathPoint(EndConnection))
                : false;

            const float ReverseHeuristic = bHasReversePrev
                ? Filter.GetHeuristicCost(
                    FRailroadGraphAStarPathPoint(PrevTrackConnFromEnd),
                    FRailroadGraphAStarPathPoint(BeginningConnection))
                : 0.0f;

            const float ReverseTraversal = bHasReversePrev
                ? Filter.GetTraversalCost(
                    FRailroadGraphAStarPathPoint(PrevTrackConnFromEnd),
                    FRailroadGraphAStarPathPoint(BeginningConnection))
                : 0.0f;

            const bool bReverseAllowed = bHasReversePrev
                ? Filter.IsTraversalAllowed(
                    FRailroadGraphAStarPathPoint(PrevTrackConnFromEnd),
                    FRailroadGraphAStarPathPoint(BeginningConnection))
                : false;

            GInspectedTrackText = FString::Printf(
                TEXT(
                    "Looking at Track: %s | Ptr: %p\n"
                    "Length: %f\n"
                    "Beginning [0]: %p\n"
                    "End [1]: %p\n"
                    "Forward PrevConn: %p%s\n"
                    "Heuristics: %f\n"
                    "Traversal: %f\n"
                    "IsTraversalAllowed: %d\n"
                    "Reverse PrevConn: %p%s\n"
                    "Rev Heuristics: %f\n"
                    "Rev Traversal: %f\n"
                    "Rev IsTraversalAllowed: %d\n"
                ),
                *nTrack->GetName(),
                nTrack,
                nTrack->GetLength(),
                BeginningConnection,
                EndConnection,
                PrevTrackConnFromBeginning,
                bHasForwardPrev ? TEXT("") : TEXT(" (kein vorheriger Track)"),
                ForwardHeuristic,
                ForwardTraversal,
                bForwardAllowed ? 1 : 0,
                PrevTrackConnFromEnd,
                bHasReversePrev ? TEXT("") : TEXT(" (kein vorheriger Track)"),
                ReverseHeuristic,
                ReverseTraversal,
                bReverseAllowed ? 1 : 0
            );

            // Bestehende Blockvisualisierung beibehalten.
            if (lastTrack != nTrack)
            {
                if (lastTrack &&
                    lastTrack != nTrack &&
                    !IsManuallySelectedTrack(lastTrack) &&
                    !IsTrainPathTrack(lastTrack))
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

            if (lastTrack)
            {
                AFGBuildableRailroadTrack* PreviousTrack = lastTrack;

                if (!IsManuallySelectedTrack(PreviousTrack) &&
                    !IsTrainPathTrack(PreviousTrack))
                {
                    PreviousTrack->StopBlockVisualization();
                }

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

        const FTrainPathingConfigStruct Config =
            FTrainPathingConfigStruct::GetActiveConfig(HUD);

        if (!Config.Debug.EnableDebugHud)
        {
            ClearTrackDebugVisualization();
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
            1.5f,   // Scale
            false   // bDontScale
        );
        // 2. Cyan Text zeichnen
        HUD->DrawText(
            GInspectedTrackText,
            FLinearColor(0.0f, 1.0f, 1.0f, 1.0f),
            ScreenX,
            ScreenY,
            Font,
            1.5f,
            false
        );

        if (Config.Debug.EnableManualTrackSelection &&
            GManuallySelectedTracks.Num() > 0)
        {
            const FString ManualSelectionText = FString::Printf(
                TEXT(
                    "Manual Track Selection\n"
                    "Tracks: %d\n"
                    "Accumulated penalty: %.4f\n"
                    "Minimum segment penalty: %.4f\n"
                    "Maximum segment penalty: %.4f\n"
                    "Accumulated penalty with current Train: %.4f"
                ),
                GManuallySelectedTracks.Num(),
                GManualSelectionAccumulatedPenalty,
                GManualSelectionMinimumPenalty,
                GManualSelectionMaximumPenalty,
                GManualSelectionAccumulatedPenaltyTrain
            );

            const float SelectionScreenX = 50.0f;
            const float SelectionScreenY = 500.0f;

            HUD->DrawText(
                ManualSelectionText,
                FLinearColor::Black,
                SelectionScreenX + 1.0f,
                SelectionScreenY + 1.0f,
                Font,
                1.5f,
                false
            );

            HUD->DrawText(
                ManualSelectionText,
                FLinearColor(1.0f, 0.8f, 0.1f, 1.0f),
                SelectionScreenX,
                SelectionScreenY,
                Font,
                1.5f,
                false
            );
        }
    }
}

static AFGRailroadVehicle* GetPlayerRailroadVehicle(
    AFGPlayerController* PlayerController)
{
    if (!IsValid(PlayerController))
    {
        return nullptr;
    }

    APawn* PlayerPawn = PlayerController->GetPawn();

    if (AFGRailroadVehicle* RailroadVehicle =
        Cast<AFGRailroadVehicle>(PlayerPawn))
    {
        return RailroadVehicle;
    }

    AFGCharacterPlayer* Character =
        Cast<AFGCharacterPlayer>(PlayerPawn);

    if (!IsValid(Character) ||
        !IsValid(PlayerController->GetWorld()))
    {
        return nullptr;
    }

    for (TActorIterator<AFGLocomotive> Iterator(
        PlayerController->GetWorld());
        Iterator;
        ++Iterator)
    {
        AFGLocomotive* Locomotive = *Iterator;

        if (IsValid(Locomotive) &&
            Locomotive->GetDriver() == Character)
        {
            return Locomotive;
        }
    }

    return nullptr;
}

static bool IsTrainFacingReverseOnTrack(
    AFGRailroadVehicle* Vehicle)
{
    if (!IsValid(Vehicle))
    {
        return false;
    }

    AFGLocomotive* Locomotive =
        Cast<AFGLocomotive>(Vehicle);

    if (!IsValid(Locomotive) ||
        !Vehicle->GetTrackPosition().IsValid())
    {
        return false;
    }

    AFGBuildableRailroadTrack* Track =
        Vehicle->GetTrackPosition().Track.Get();

    if (!IsValid(Track))
    {
        return false;
    }

    USplineComponent* Spline =
        Track->GetSplineComponent();

    if (!IsValid(Spline))
    {
        return false;
    }

    const FRailroadTrackPosition& TrackPosition =
        Vehicle->GetTrackPosition();

    const FVector SplineDirection =
        Spline->GetDirectionAtDistanceAlongSpline(
            TrackPosition.Offset,
            ESplineCoordinateSpace::World)
        .GetSafeNormal();

    const FVector LocomotiveDirection =
        Locomotive->GetActorForwardVector().GetSafeNormal();

    if (SplineDirection.IsNearlyZero() ||
        LocomotiveDirection.IsNearlyZero())
    {
        return false;
    }

    /*
     * This is based only on the physical orientation of the
     * locomotive, not on its movement direction.
     */
    return FVector::DotProduct(
        LocomotiveDirection,
        SplineDirection) < 0.0f;
}

static void RecalculateManualSelectionStatistics(
    AFGPlayerController* PlayerController)
{
    if (!IsValid(PlayerController))
    {
        return;
    }

    const FTrainPathingConfigStruct Config =
        FTrainPathingConfigStruct::GetActiveConfig(PlayerController);

    FRailroadGraphAStarFilter OriginalFilter;
    FFactorioRailroadAStarFilter Filter(
        OriginalFilter,
        Config);

    AFGRailroadVehicle* MarkingVehicle =
        GetPlayerRailroadVehicle(PlayerController);

    if (!IsValid(MarkingVehicle))
    {
        return;
    }


    GManualSelectionAccumulatedPenalty = 0.0f;
    GManualSelectionAccumulatedPenaltyTrain = 0.0f;
    GManualSelectionMinimumPenalty = 0.0f;
    GManualSelectionMaximumPenalty = 0.0f;

    bool bHasPenalty = false;

    for (const Trackdir& Trackd :
        GManuallySelectedTracks)
    {
        if (!Trackd.Track.IsValid())
        {
            continue;
        }

        AFGBuildableRailroadTrack* TrackActor =
            Trackd.Track.Get();

        if (!IsValid(TrackActor))
        {
            continue;
        }

        const bool bTrackIsReversed =
            Trackd.reversed;
        /*
        UE_LOG(
            train_pathing_debug,
            Warning,
            TEXT(
                "Manual track evaluated: %s, Reversed=%d"
            ),
            *TrackActor->GetName(),
            bTrackIsReversed ? 1 : 0
        );
        */

        /*
         * RearConnection und FrontConnection sind beide Enden desselben
         * Tracks (TrackActor). Nach der A*-Semantik darf aber niemals
         * RearConnection direkt als StartNodeRef verwendet werden, wenn
         * FrontConnection der EndNodeRef ist - beide müssten sonst auf
         * demselben Track liegen, was nur beim EndNodeRef korrekt ist.
         *
         * Der tatsächliche StartNodeRef liegt physisch auf dem VORHERIGEN
         * Track, gemäß der Fahrtrichtung des Zuges (Orientierung, nicht
         * Bewegungsrichtung). Dieser wird über die reale Verbindung
         * RearConnection->GetConnection() ermittelt, die zur
         * gegenüberliegenden Connection auf dem vorangehenden Track führt.
         *
         * Ist kein vorheriger Track vorhanden (z.B. Streckenende oder erster
         * Track ohne Nachbarn), wird auf RearConnection als Fallback
         * zurückgegriffen und IgnoredStart gesetzt, damit A*-Startlogik
         * korrekt greift.
         */
        UFGRailroadTrackConnectionComponent* RearConnection =
            bTrackIsReversed
            ? TrackActor->GetConnection(1)
            : TrackActor->GetConnection(0);

        UFGRailroadTrackConnectionComponent* FrontConnection =
            bTrackIsReversed
            ? TrackActor->GetConnection(0)
            : TrackActor->GetConnection(1);

        if (!IsValid(RearConnection) ||
            !IsValid(FrontConnection))
        {
            continue;
        }

        UFGRailroadTrackConnectionComponent* PreviousTrackConnection =
            RearConnection->GetConnection();

        const bool bHasPreviousTrack =
            IsValid(PreviousTrackConnection);

        FRailroadGraphAStarPathPoint StartPoint(
            bHasPreviousTrack
                ? PreviousTrackConnection
                : RearConnection,
            true);

        FRailroadGraphAStarPathPoint GoalPoint(
            FrontConnection);

        const float TrackPenalty_Train =
            Filter.GetTraversalCost(
                StartPoint,
                GoalPoint);

        const float TrackPenalty =
            Filter.GetTraversalCost(
                StartPoint,
                GoalPoint,
                GLastEnteredManualTrain.Get());

        GManualSelectionAccumulatedPenalty +=
            TrackPenalty;

        GManualSelectionAccumulatedPenaltyTrain +=
            TrackPenalty_Train;

        if (!bHasPenalty)
        {
            GManualSelectionMinimumPenalty = TrackPenalty;
            GManualSelectionMaximumPenalty = TrackPenalty;
            bHasPenalty = true;
        }
        else
        {
            GManualSelectionMinimumPenalty =
                FMath::Min(
                    GManualSelectionMinimumPenalty,
                    TrackPenalty);

            GManualSelectionMaximumPenalty =
                FMath::Max(
                    GManualSelectionMaximumPenalty,
                    TrackPenalty);
        }
    }
}


static void UpdateManualTrackSelection(
    AFGPlayerController* PlayerController)
{
    if (!IsValid(PlayerController))
    {
        return;
    }

    const FTrainPathingConfigStruct Config =
        FTrainPathingConfigStruct::GetActiveConfig(PlayerController);

    if (!Config.Debug.EnableDebugHud ||
        !Config.Debug.EnableManualTrackSelection)
    {
        ClearManualTrackSelection();
        return;
    }

    if (GManuallySelectedTracks.Num() > 0 &&
        !GLastEnteredManualTrain.IsValid())
    {
        ClearManualTrackSelection();
    }

    AFGTrain* CurrentTrain =
        GetPlayerTrain(PlayerController);

    if (!IsValid(CurrentTrain))
    {
        return;
    }

    AFGRailroadVehicle* CurrentVehicle =
        GetPlayerRailroadVehicle(PlayerController);

    if (!IsValid(CurrentVehicle))
    {
        return;
    }

    AFGBuildableRailroadTrack* CurrentTrack =
        CurrentVehicle->GetTrackPosition().Track.Get();

    if (!IsValid(CurrentTrack))
    {
        return;
    }

    const bool bEnteredDifferentTrain =
        GLastEnteredManualTrain.Get() != CurrentTrain;

    if (CurrentTrain->mTrainStatus != ETrainStatus::TS_ManualDriving) {
        ClearManualTrackSelection();
        GLastEnteredManualTrain = CurrentTrain;
        return;
    }
    /*
     * Beim Betreten eines anderen Zuges bleibt die Auswahl erhalten,
     * sofern der neue Zug auf einem bereits ausgewählten Track steht.
     * Andernfalls beginnt eine neue Auswahl.
     */
    if (bEnteredDifferentTrain)
    {
        if (!ContainsTrack(
            GManuallySelectedTracks,
            CurrentTrack))
        {
            ClearManualTrackSelection();
        }

        GLastEnteredManualTrain = CurrentTrain;
    }

    if (!ContainsTrack(
        GManuallySelectedTracks,
        CurrentTrack))
    {
        const bool bTrainFacesReverse =
            IsTrainFacingReverseOnTrack(CurrentVehicle);

        UE_LOG(
            train_pathing_debug,
            Warning,
            TEXT(
                "Manual track recorded: %s, "
                "Forward=%.3f, Reversed=%d"
            ),
            *CurrentTrack->GetName(),
            CurrentVehicle->GetTrackPosition().Forward,
            bTrainFacesReverse ? 1 : 0
        );

        GManuallySelectedTracks.Add(
            Trackdir(
                CurrentTrack,
                bTrainFacesReverse));
        CurrentTrack->ShowBlockVisualization();
    }
    RecalculateManualSelectionStatistics(PlayerController);
}

namespace TrainPathingDebug
{
    void Startup()
    {
        SUBSCRIBE_UOBJECT_METHOD(
            AFGPlayerController,
            PlayerTick,
            [](auto& scope, AFGPlayerController* self, float DeltaSeconds)
            {
                scope(self, DeltaSeconds);

                if (self &&
                    self->IsLocalController())
                {
                    UpdateInspectedTrackData(self, DeltaSeconds);
                    UpdatePlayerTrainPathVisualization(self);
                    UpdateManualTrackSelection(self);
                }
            });

        SUBSCRIBE_UOBJECT_METHOD(
            AHUD,
            DrawHUD,
            [](auto& scope, AHUD* self)
            {
                scope(self);

                if (self)
                {
                    DrawTrackHUD_Canvas(self);
                }
            });

        UE_LOG(
            train_pathing_debug,
            Verbose,
            TEXT("Hooked PlayerTick for Track Debug HUD"));
    }

    void Shutdown()
    {
        ClearTrackDebugVisualization();
    }
}

#endif