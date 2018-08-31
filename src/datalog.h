#ifndef DATALOG_H
#define DATALOG_H

#include "ChRt.h"

extern THD_WORKING_AREA(waDatalogThread, 128);

extern THD_FUNCTION(DatalogThread, arg);

#endif
