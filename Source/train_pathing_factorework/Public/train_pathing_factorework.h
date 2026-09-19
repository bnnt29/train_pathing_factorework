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

DECLARE_LOG_CATEGORY_EXTERN(train_pathing, Verbose, All);

TRAIN_PATHING_FACTOREWORK_API float CountVehiclesOnTrack(
    AFGBuildableRailroadTrack* Track,
    const FTrainPathingConfigStruct& Config,
    const AFGTrain* IgnoredTrain = nullptr);

TRAIN_PATHING_FACTOREWORK_API float CountStationPlatforms(
    UFGRailroadTrackConnectionComponent* RailroadConnection,
    const FTrainPathingConfigStruct& Config);

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
        AFGBuildableRailroadTrack* Track,
        const AFGTrain* IgnoredTrain = nullptr) const;
};

class Ftrain_pathing_factoreworkModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};