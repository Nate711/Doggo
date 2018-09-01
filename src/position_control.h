#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "ChRT.h"
#include "ODriveArduino.h"

extern THD_WORKING_AREA(waPositionControlThread, 512);
extern THD_FUNCTION(PositionControlThread, arg);

void HipAngleToCartesian(float alpha, float beta, float& x, float& y);
void GetGamma(float L, float theta, float& gamma);
void LegParamsToHipAngles(float L, float theta, float& alpha, float& beta);
void LegParamsToCartesian(float L, float theta, float& x, float& y);
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta);
void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma);
void SinTrajectory (float t, struct GaitParams params, float gaitOffset, float& x, float& y);
void CoupledMoveLeg(ODriveArduino& odrive, float t, struct GaitParams params, float gait_offset, float leg_direction, struct LegGain gains);
void SinTrajectoryPosControl();
void gait(struct GaitParams params, float leg0_offset, float leg1_offset, float leg2_offset, float leg3_offset, struct LegGain gains);
void trot();
void pronk();
void bound();

struct GaitParams {
    float stanceHeight = 0.18; // Desired height of body from ground during walking (m)
    float downAMP = 0.00; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    float upAMP = 0.06; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    float flightPercent = 0.6; // Portion of the gait time should be doing the down portion of trajectory
    float stepLength = 0.0; // Length of entire step (m)
    float FREQ = 1.0; // Frequency of one gait cycle (Hz)
};

#endif
