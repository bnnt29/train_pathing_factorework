// Copyright Epic Games, Inc. All Rights Reserved.

#include "train_pathing_factorework.h"

//#define LOCTEXT_NAMESPACE "Ftrain_pathing_factoreworkModule"

#include "Patching/NativeHookManager.h"

#include "RailroadNavigation.h"
#include "FGLocomotive.h"
#include "Buildables/FGBuildableRailroadStation.h"
#include "FGRailroadTrackConnectionComponent.h"

DEFINE_LOG_CATEGORY(train_pathing);

/** Used as GetHeuristicCost's multiplier. */
void GetHeuristicScaleHook(auto& scope, const FRailroadGraphAStarFilter* self) {
	UE_LOG(train_pathing, Verbose, TEXT("Orig_Scale = %f"), scope(self));
	scope.Override(1.5f);
}

/** Estimate of cost from startNodeRef to endNodeRef. */
void GetHeuristicCostHook(auto& scope, const FRailroadGraphAStarFilter* self, const FRailroadGraphAStarPathPoint& startNodeRef, const FRailroadGraphAStarPathPoint& endNodeRef) {
	//UE_LOG(train_pathing, Verbose, TEXT("Hstart = %s"), *startNodeRef.TrackConnection->GetTrack()->GetTargetLocation().ToString());
	//UE_LOG(train_pathing, Verbose, TEXT("Hend = %s"), *endNodeRef.TrackConnection->GetTrack()->GetTargetLocation().ToString());
	UE_LOG(train_pathing, Verbose, TEXT("Orig_Heuristic = %f"), scope(self, startNodeRef, endNodeRef));
	scope.Override(1.25f);
}

/** Real cost of traveling from startNodeRef directly to endNodeRef */
void GetTraversalCostHook(auto& scope, const FRailroadGraphAStarFilter* self, const FRailroadGraphAStarPathPoint& startNodeRef, const FRailroadGraphAStarPathPoint& endNodeRef) {
	//UE_LOG(train_pathing, Verbose, TEXT("Tstart = %s"), *startNodeRef.TrackConnection->GetTrack()->GetTargetLocation().ToString());
	//UE_LOG(train_pathing, Verbose, TEXT("Tend = %s"), *endNodeRef.TrackConnection->GetTrack()->GetTargetLocation().ToString());
	UE_LOG(train_pathing, Verbose, TEXT("Orig_Traversal = %f"), scope(self, startNodeRef, endNodeRef));
	scope.Override(1.75f);
}

/** Whether traversing given edge is allowed. */
void IsTraversalAllowedHook(auto& scope, const FRailroadGraphAStarFilter* self, const FRailroadGraphAStarPathPoint& nodeA, const FRailroadGraphAStarPathPoint& nodeB) {
	//UE_LOG(train_pathing, Verbose, TEXT("Astart = %s"), *nodeA.TrackConnection->GetTrack()->GetTargetLocation().ToString());
	//UE_LOG(train_pathing, Verbose, TEXT("Aend = %s"), *nodeB.TrackConnection->GetTrack()->GetTargetLocation().ToString());
	UE_LOG(train_pathing, Verbose, TEXT("Orig_IsTraversal = %d"), scope(self, nodeA, nodeB));
	scope.Override(true);
}

/** Whether to accept solutions that do not reach the goal.
void WantsPartialSolutionHook(auto& scope) {
	scope.Override(false);
} */


void FindPathSyncHook(auto& scope, AFGLocomotive* locomotive,
	AFGBuildableRailroadStation* station,
	FRailroadGraphAStarFilter filter)
{
	filter.
	FRailroadPathFindingResult orig_result = scope(locomotive, station, filter);
	switch (orig_result.Result)
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
	//orig_result.Result = ERailroadPathFindingResult::RPFR_Unreachable;
	scope.Override(orig_result);
}


void Ftrain_pathing_factoreworkModule::StartupModule()
{
	UE_LOG(train_pathing, Verbose, TEXT("Starting Train Pathing1"));
	if (!WITH_EDITOR) {
		UE_LOG(train_pathing, Verbose, TEXT("Hooking Path Finder"));
		SUBSCRIBE_METHOD(FRailroadNavigation::FindPathSync, [](auto& scope, AFGLocomotive* locomotive,
			AFGBuildableRailroadStation* station,
			FRailroadGraphAStarFilter filter) {
				UE_LOG(train_pathing, Verbose, TEXT("Train pathing"));
				FindPathSyncHook(scope, locomotive, station, filter);
				UE_LOG(train_pathing, Verbose, TEXT("Train pathing finished"));
			});
		SUBSCRIBE_METHOD(FRailroadGraphAStarFilter::GetHeuristicScale, [](auto& scope, const FRailroadGraphAStarFilter* self) {
			GetHeuristicScaleHook(scope, self);
			});
		SUBSCRIBE_METHOD(FRailroadGraphAStarFilter::GetHeuristicCost, [](auto& scope, const FRailroadGraphAStarFilter* self, const FRailroadGraphAStarPathPoint& startNodeRef, const FRailroadGraphAStarPathPoint& endNodeRef) {
			GetHeuristicCostHook(scope, self, startNodeRef, endNodeRef);
			});
		SUBSCRIBE_METHOD(FRailroadGraphAStarFilter::GetTraversalCost, [](auto& scope, const FRailroadGraphAStarFilter* self, const FRailroadGraphAStarPathPoint& startNodeRef, const FRailroadGraphAStarPathPoint& endNodeRef) {
			GetTraversalCostHook(scope, self, startNodeRef, endNodeRef);
			});
		SUBSCRIBE_METHOD(FRailroadGraphAStarFilter::IsTraversalAllowed, [](auto& scope, const FRailroadGraphAStarFilter* self, const FRailroadGraphAStarPathPoint& nodeA, const FRailroadGraphAStarPathPoint& nodeB) {
			IsTraversalAllowedHook(scope, self, nodeA, nodeB);
			});
		/*SUBSCRIBE_METHOD(FRailroadGraphAStarFilter::WantsPartialSolution, [](auto& scope, const FRailroadGraphAStarFilter* self) {
			WantsPartialSolutionHook(scope);
			});*/
		UE_LOG(train_pathing, Verbose, TEXT("Hooked Path Finder"));
	}
	else {
		UE_LOG(train_pathing, Verbose, TEXT("Not Hooked!"));
	}
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void Ftrain_pathing_factoreworkModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(Ftrain_pathing_factoreworkModule, train_pathing_factorework)