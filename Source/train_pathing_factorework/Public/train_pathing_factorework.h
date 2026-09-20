// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Logging/LogMacros.h"
#include "Modules/ModuleManager.h"

#include "GraphAStar.h"
#include "RailroadNavigation.h"
#include "TrainPathingConfigStruct.h"
#include "FGTrain.h"

class AFGBuildableRailroadTrack;
class AFGLocomotive;
class AFGBuildableRailroadStation;
class UFGRailroadTrackConnectionComponent;

DECLARE_LOG_CATEGORY_EXTERN(train_pathing, Verbose, All);

TRAIN_PATHING_FACTOREWORK_API float CountVehiclesOnTrack(
    AFGBuildableRailroadTrack* Track,
    const FTrainPathingConfigStruct& Config,
    const AFGTrain* IgnoredTrain = nullptr);

TRAIN_PATHING_FACTOREWORK_API float CountStationPlatforms(
    UFGRailroadTrackConnectionComponent* RailroadConnection,
    const FTrainPathingConfigStruct& Config);

/**
 * Runs the custom A* pathfinding starting from an arbitrary track connection (instead of always
 * starting from the locomotive's own position), allowing callers to plan only a suffix of a route
 * (e.g. from a signal further down the line) while keeping the untouched prefix of an existing path.
 */
TRAIN_PATHING_FACTOREWORK_API FRailroadPathFindingResult FindPathSyncFrom(
    AFGLocomotive* locomotive,
    UFGRailroadTrackConnectionComponent* startConnection,
    bool bIgnoredStart,
    AFGBuildableRailroadStation* station,
    FRailroadGraphAStarFilter filter);

struct TRAIN_PATHING_FACTOREWORK_API FFactorioRailroadAStarFilter :
    public FRailroadGraphAStarFilter
{
    const FRailroadGraphAStarFilter& BaseFilter;
    FTrainPathingConfigStruct Config;

    FFactorioRailroadAStarFilter(
        const FRailroadGraphAStarFilter& InBase,
        const FTrainPathingConfigStruct& InConfig);

    float GetHeuristicScale() const;

    float GetHeuristicCost(
        const FRailroadGraphAStarPathPoint& StartNodeRef,
        const FRailroadGraphAStarPathPoint& EndNodeRef) const;

    bool IsTraversalAllowed(
        const FRailroadGraphAStarPathPoint& NodeA,
        const FRailroadGraphAStarPathPoint& NodeB) const;

    float GetTraversalCost(
        const FRailroadGraphAStarPathPoint& StartNodeRef,
        const FRailroadGraphAStarPathPoint& EndNodeRef,
        const AFGTrain* IgnoredTrain = nullptr) const;

    bool WantsPartialSolution() const;
    bool ShouldIncludeStartNodeInPath() const;

private:
    float CalculateFactorioPenalty(
        const FRailroadGraphAStarPathPoint& StartNodeRef,
        const FRailroadGraphAStarPathPoint& EndNodeRef,
        AFGBuildableRailroadTrack* Track,
        const AFGTrain* IgnoredTrain) const;
};

class Ftrain_pathing_factoreworkModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};