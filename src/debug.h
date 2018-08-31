#ifndef DEBUG_H
#define DEBUG_H

#include "ChRt.h"

extern THD_WORKING_AREA(waPrintDebugThread, 256);
extern THD_FUNCTION(PrintDebugThread, arg);

#endif
