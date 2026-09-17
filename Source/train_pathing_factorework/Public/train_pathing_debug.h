// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Logging/LogMacros.h"

DECLARE_LOG_CATEGORY_EXTERN(train_pathing_debug, Verbose, All);

namespace TrainPathingDebug
{
    void Startup();
    void Shutdown();
}