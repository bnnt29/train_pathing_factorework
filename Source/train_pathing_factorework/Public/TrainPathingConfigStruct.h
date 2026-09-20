#pragma once
#include "CoreMinimal.h"
#include "Configuration/ConfigManager.h"
#include "Engine/Engine.h"
#include "TrainPathingConfigStruct.generated.h"

struct FTrainPathingConfigStruct_Debug;
struct FTrainPathingConfigStruct_Trains;
struct FTrainPathingConfigStruct_Platforms;
struct FTrainPathingConfigStruct_Tracks;
struct FTrainPathingConfigStruct_Other;
struct FTrainPathingConfigStruct_Trains_SelfDriving;
struct FTrainPathingConfigStruct_Platforms_CargoPlatform;
struct FTrainPathingConfigStruct_Platforms_Docking;
struct FTrainPathingConfigStruct_Tracks_Thresholds;

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Trains_SelfDriving {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    float NoTimeTablePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float InvalidNextStopPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float InvalidLocomotivePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float NoPathPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float StationUnreachablePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float SignalUnreachablePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float LongWaitPenalty{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Platforms_CargoPlatform {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    float WaitingPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float LoadingPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float TransferPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float CompletePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float ConditionPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float IdlePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float CargoBasePenalty{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Platforms_Docking {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    float ReadyPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float CompletePenalty{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Tracks_Thresholds {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    float TightCurveRadiusThreshold{};

    UPROPERTY(BlueprintReadWrite)
    float ClimbingSlopeThreshold{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Debug {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    bool EnableDebugHud{};

    UPROPERTY(BlueprintReadWrite)
    bool EnableVerboseLogging{};

    UPROPERTY(BlueprintReadWrite)
    bool UseOriginalPathFinding{};

    UPROPERTY(BlueprintReadWrite)
    bool EnableManualTrackSelection{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Trains {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    float LocomotiveForwardPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float LocomotiveReversedPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float PlayerDrivenTrainPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float FreightWagonPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float DockedVehiclePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float DerailedVehiclePenalty{};

    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Trains_SelfDriving SelfDriving{};

    UPROPERTY(BlueprintReadWrite)
    float PathReservationPenalty{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Platforms {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    float StationBasePenalty{};

    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Platforms_CargoPlatform CargoPlatform{};

    UPROPERTY(BlueprintReadWrite)
    float EmptyPlatformPenalty{};

    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Platforms_Docking Docking{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Tracks {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Tracks_Thresholds Thresholds{};

    UPROPERTY(BlueprintReadWrite)
    float TightCurvePenalty{};

    UPROPERTY(BlueprintReadWrite)
    float ClimbingPenalty{};

    UPROPERTY(BlueprintReadWrite)
    float DescendingSlopeBonus{};
};

USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct_Other {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    float BasePenaltyScale{};
};

/* Struct generated from Mod Configuration Asset '/train_pathing_factorework/TrainPathingConfig' */
USTRUCT(BlueprintType)
struct FTrainPathingConfigStruct {
    GENERATED_BODY()
public:
    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Debug Debug{};

    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Trains Trains{};

    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Platforms Platforms{};

    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Tracks Tracks{};

    UPROPERTY(BlueprintReadWrite)
    FTrainPathingConfigStruct_Other Other{};

    /* Retrieves active configuration value and returns object of this struct containing it */
    static FTrainPathingConfigStruct GetActiveConfig(UObject* WorldContext) {
        FTrainPathingConfigStruct ConfigStruct{};
        FConfigId ConfigId{"train_pathing_factorework", ""};
        if (const UWorld* World = GEngine->GetWorldFromContextObject(WorldContext, EGetWorldErrorMode::ReturnNull)) {
            UConfigManager* ConfigManager = World->GetGameInstance()->GetSubsystem<UConfigManager>();
            ConfigManager->FillConfigurationStruct(ConfigId, FDynamicStructInfo{FTrainPathingConfigStruct::StaticStruct(), &ConfigStruct});
        }
        return ConfigStruct;
    }
};

