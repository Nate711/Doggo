#ifndef JUMP_H
#define JUMP_H

#include "ODriveArduino.h"
void TrajectoryJump(float t, float launchTime, float stanceHeight,
                    float downAMP, float& x, float& y);
void StartJump(float start_time_s);
void ExecuteJump();
void CommandAllLegs(float theta, float gamma, LegGain gains);

#endif
