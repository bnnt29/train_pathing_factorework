// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Logging/LogMacros.h"
#include "Modules/ModuleManager.h"

// Declare the category so every file including this header recognizes it
DECLARE_LOG_CATEGORY_EXTERN(train_pathing, Verbose, All);

class Ftrain_pathing_factoreworkModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
   /* void FindPathSyncInternalHook(auto& scope, const FRailroadGraphAStarPathPoint& start,
        const FRailroadGraphAStarPathPoint& end,
        FRailroadGraphAStarFilter filter,
        TArray< FRailroadGraphAStarPathPoint >& out_pathPoints);*/  
};