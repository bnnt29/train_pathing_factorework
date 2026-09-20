#pragma once

#include "Logging/LogMacros.h"

DECLARE_LOG_CATEGORY_EXTERN(LogSeamlessTrainPathing, Log, Log);

namespace FSeamlessTrainPathingModule
{
	void Startup();
	void Shutdown();
};