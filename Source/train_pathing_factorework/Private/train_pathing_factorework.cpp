// Copyright Epic Games, Inc. All Rights Reserved.

#include "train_pathing_factorework.h"

#include "Patching/NativeHookManager.h"

#include "RailroadNavigation.h"
#include "FGLocomotive.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "FGRailroadTrackConnectionComponent.h"
#include "Buildables/FGBuildableRailroadTrack.h"
#include "Buildables/FGBuildableTrainPlatform.h"
#include "FGTrainPlatformConnection.h"
#include "GraphAStar.h"

DEFINE_LOG_CATEGORY(train_pathing);

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

// State variables to cache the inspected track
static FString GInspectedTrackText = TEXT("Looking at Track: [None]");
static float GTrackTraceTimer = 0.0f;
static constexpr float TRACE_INTERVAL = 0.2f; // Run raycast every 0.2s
static AFGBuildableRailroadTrack* lastTrack = nullptr;
static TArray<TWeakObjectPtr<AActor>> GConnectionMarkerActors;
static TWeakObjectPtr<AFGBuildableRailroadTrack> GMarkerTrack;

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
    if (!Connection)
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
    if (!World || !Connection)
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

    if (!MarkerActor)
    {
        return nullptr;
    }

    UStaticMeshComponent* MarkerMesh =
        NewObject<UStaticMeshComponent>(MarkerActor);

    if (!MarkerMesh)
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

    if (!SphereMesh)
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
    if (!World || !Track)
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

        if (!Connection)
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
    if (!World || !PC || !PC->PlayerCameraManager) return nullptr;

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
static void UpdateInspectedTrackData(AFGPlayerController* FGPC, float DeltaSeconds)
{
    if (!FGPC) return;

    GTrackTraceTimer += DeltaSeconds;
    if (GTrackTraceTimer < TRACE_INTERVAL)
    {
        return;
    }
    GTrackTraceTimer = 0.0f;
    AFGBuildableRailroadTrack* nTrack = GetLookedAtRailTrack(FGPC->GetWorld(), FGPC);
    if (nTrack)
    {
        UpdateTrackConnectionMarkers(FGPC->GetWorld(), nTrack);

        GInspectedTrackText = FString::Printf(
            TEXT(
                "Looking at Track: %s | Ptr: %p\n"
                "Beginning [0]: %p\n"
                "End [1]: %p"
            ),
            *nTrack->GetName(),
            nTrack,
            nTrack->GetConnection(0),
            nTrack->GetConnection(1)
        );

        // Bestehende Blockvisualisierung beibehalten.
        if (lastTrack != nTrack)
        {
            if (lastTrack)
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
            lastTrack->StopBlockVisualization();
            lastTrack = nullptr;
        }

        GInspectedTrackText = TEXT("Looking at Track: [None]");
    }
}
static void DrawTrackHUD_Canvas(AHUD* HUD)
{
    if (!HUD) return;

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

struct FFactorioRailroadAStarFilter : public FRailroadGraphAStarFilter
{
    const FRailroadGraphAStarFilter& BaseFilter;

    FFactorioRailroadAStarFilter(const FRailroadGraphAStarFilter& InBase)
        : BaseFilter(InBase) {
    }

    float GetHeuristicScale() const {
        return BaseFilter.GetHeuristicScale();
    }

    float GetHeuristicCost(const FRailroadGraphAStarPathPoint& StartNodeRef, const FRailroadGraphAStarPathPoint& EndNodeRef) const
    {
        float OrigCost = BaseFilter.GetHeuristicCost(StartNodeRef, EndNodeRef);

        if (!StartNodeRef.TrackConnection || !EndNodeRef.TrackConnection)
        {
            return 0.0f;
        }
        float NewCost = FVector::Dist(StartNodeRef.TrackConnection->GetComponentLocation(), EndNodeRef.TrackConnection->GetComponentLocation());
        UE_LOG(train_pathing, Verbose, TEXT("Heuristic = %f <=> %f"), OrigCost, NewCost);
        return NewCost;
    }

    bool IsTraversalAllowed(const FRailroadGraphAStarPathPoint& NodeA, const FRailroadGraphAStarPathPoint& NodeB) const
    {
        bool bAllowed = BaseFilter.IsTraversalAllowed(NodeA, NodeB);
        //UE_LOG(train_pathing, Verbose, TEXT("Orig_IsTraversal = %d"), bAllowed ? 1 : 0);

        if (!NodeA.TrackConnection || !NodeB.TrackConnection)
        {
            return false;
        }

        return bAllowed;
    }

    float CalculateFactorioPenalty(UFGRailroadTrackConnectionComponent* Conn, float SegmentLength) const
    {
        float Penalty = 0.0f;

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

        if (!StartNodeRef.TrackConnection || !EndNodeRef.TrackConnection)
        {
            return 0.0f;
        }

        UFGRailroadTrackConnectionComponent* ConnB = EndNodeRef.TrackConnection;
        AFGBuildableRailroadTrack* Track = ConnB ? ConnB->GetTrack() : nullptr;
        float SegmentLength = Track ? Track->GetLength() : 1000.0f;

        // Base Cost (Rule 16: Length, slope, curvature adjustments)
        float BaseCost = SegmentLength;

        // Apply Factorio Penalty Table
        float Penalty = CalculateFactorioPenalty(ConnB, SegmentLength);
        float NewCost = BaseCost + Penalty;
        UE_LOG(train_pathing, Verbose, TEXT("Traversal = %f <=> %f"), OrigCost, NewCost);
        return NewCost;
    }

    bool WantsPartialSolution() const { return BaseFilter.WantsPartialSolution(); }
    bool ShouldIncludeStartNodeInPath() const { return BaseFilter.ShouldIncludeStartNodeInPath(); }
};


void FindPathSyncHook(auto& scope, AFGLocomotive* locomotive,
    AFGBuildableRailroadStation* station,
    FRailroadGraphAStarFilter filter)
{
    FRailroadPathFindingResult Result;
    Result.Locomotive = locomotive;
    Result.Result = ERailroadPathFindingResult::RPFR_Error;

    if (!locomotive || !station)
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

    if (!GoalConn)
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

    if (!StartConn || !GoalConn)
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
    FFactorioRailroadAStarFilter CustomFilter(filter);
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
        if (!Pt.TrackConnection) continue;
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
            if (!C)
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

            UE_LOG(train_pathing, Verbose, TEXT("  %s Pt[%d] Conn=%p Owner=%s Track=%p TrackLen=%f ConnIndex=%d ForwardOffset=%f ReverseOffset=%f"),
                *Tag, Index, C, OwnerName, T, TrackLength, ConnIndex, ForwardOffset, ReverseOffset);
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
                if (self && self->IsLocalController())
                {
                    UpdateInspectedTrackData(self, DeltaSeconds);
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
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(Ftrain_pathing_factoreworkModule, train_pathing_factorework)
