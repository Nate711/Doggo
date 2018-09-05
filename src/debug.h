#ifndef DEBUG_H
#define DEBUG_H

#include "ChRT.h"
#include "globals.h"

extern THD_WORKING_AREA(waPrintDebugThread, 256);
extern THD_FUNCTION(PrintDebugThread, arg);

void PrintODriveDebugInfo(struct ODrive odrv);

#endif
