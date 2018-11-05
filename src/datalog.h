#ifndef DATALOG_H
#define DATALOG_H

#include "ChRt.h"

extern THD_WORKING_AREA(waDatalogThread, 2048);

extern THD_FUNCTION(DatalogThread, arg);

#endif
