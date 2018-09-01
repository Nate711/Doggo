#ifndef JUMP_H
#define JUMP_H

void TrajectoryJump(float t, float launchTime, float stanceHeight,
                    float downAMP, float& x, float& y);
void StartJump(float start_time_s);
bool ShouldExecuteJump();
void ExecuteJump();

#endif
