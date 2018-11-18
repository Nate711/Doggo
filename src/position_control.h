#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "ChRt.h"
#include "ODriveArduino.h"

extern THD_WORKING_AREA(waPositionControlThread, 512);
extern THD_FUNCTION(PositionControlThread, arg);

void GetGamma(float L, float theta, float& gamma);
void LegParamsToCartesian(float L, float theta, float& x, float& y);
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta);
void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma);
void SinTrajectory (float t, struct GaitParams params, float gaitOffset, float& x, float& y);
void CoupledMoveLeg(ODriveArduino& odrive, float t, struct GaitParams params, float gait_offset, float leg_direction, struct LegGain gains);
bool IsValidGaitParams(struct GaitParams params);
bool IsValidLegGain(struct LegGain gain);
void SinTrajectoryPosControl();
void gait(struct GaitParams params, float leg0_offset, float leg1_offset, float leg2_offset, float leg3_offset, struct LegGain gains);
void TransitionToDance();
void TransitionToWalk();
void TransitionToTrot();
void TransitionToPronk();
void TransitionToBound();
void TransitionToRotate();
void TransitionToHop();
void PrintGaitParams();
void SetODriveCurrentLimits(float limit);
void test();
void hop(struct GaitParams params);
void reset();
void CommandAllLegs(float theta, float gamma, struct LegGain gains);

enum States {
    STOP = 0,
    TROT = 1,
    BOUND = 2,
    WALK = 3,
    PRONK = 4,
    JUMP = 5,
    DANCE = 6,
    HOP = 7,
    TEST = 8,
    ROTATE = 9,
    FLIP = 10,
    RESET = 11
};

void UpdateStateGaitParams(States curr_state);

extern States state;

struct GaitParams {
    float stance_height = 0.18; // Desired height of body from ground during walking (m)
    float down_amp = 0.00; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    float up_amp = 0.06; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    float flight_percent = 0.6; // Portion of the gait time should be doing the down portion of trajectory
    float step_length = 0.0; // Length of entire step (m)
    float freq = 1.0; // Frequency of one gait cycle (Hz)
};

extern struct GaitParams state_gait_params[12];
extern struct LegGain gait_gains;
extern long rotate_start; // milliseconds when rotate was commanded

#endif
