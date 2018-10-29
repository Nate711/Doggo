#ifndef IMU_H
#define IMU_H

#include "ChRt.h"

extern THD_WORKING_AREA(waIMUThread, 4096);

extern THD_FUNCTION(IMUThread, arg);

void IMUTarePitch();

#endif
