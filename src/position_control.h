#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "ChRt.h"
#include "ODriveArduino.h"

extern THD_WORKING_AREA(waPositionControlThread, 512);
extern THD_FUNCTION(PositionControlThread, arg);

void HipAngleToCartesian(float alpha, float beta, float& x, float& y);
void GetGamma(float L, float theta, float& gamma);
void LegParamsToHipAngles(float L, float theta, float& alpha, float& beta);
void LegParamsToCartesian(float L, float theta, float& x, float& y);
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta);
void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma);
void SinTrajectory(float t, float FREQ, float gaitOffset, float stanceHeight, float flightPercent, float stepLength, float upAMP, float downAMP, float& x, float& y);
void CartesianToEncoder(float x, float y, float leg_direction, float sign, float& enc0, float& enc1);
void CoupledMoveLeg(ODriveArduino& odrive, float t, float FREQ, float gait_offset, float stanceHeight, float flightPercent, float stepLength, float upAMP, float downAMP, float leg_direction);
void SinTrajectoryPosControl();
void trot();
void pronk();
void bound();

#endif
