#ifndef IMU_H
#define IMU_H

#include "ChRt.h"

extern THD_WORKING_AREA(waIMUThread, 1024);

extern THD_FUNCTION(IMUThread, arg);

#endif
