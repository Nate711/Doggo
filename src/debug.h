#ifndef DEBUG_H
#define DEBUG_H

#include "ChRT.h"
#include "globals.h"

extern THD_WORKING_AREA(waPrintDebugThread, 1024);
extern THD_FUNCTION(PrintDebugThread, arg);

void PrintODriveDebugInfo(struct ODrive odrv);
void PrintForces(struct ODrive odrv0, struct ODrive odrv1, struct ODrive odrv2, struct ODrive odrv3);

#endif
