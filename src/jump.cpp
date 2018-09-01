#ifndef JUMP_H
#define JUMP_H

#include "jump.h"
#include "ODriveArduino.h"
#include "globals.h"
#include "position_control.h"

// Privates
bool execute_jump_ = false;
float start_time_ = 0.0f;

/**
 * Tell the position control thread to do the jump
 * @param start_time_s The timestamp of when the jump command was sent
 */
void StartJump(float start_time_s) {
    start_time_ = start_time_s;
    execute_jump_ = true;
}
/**
 * Query if a jump should be executed
 */
bool ShouldExecuteJump() {
    return execute_jump_;
}

/**
* Linear increase in height for jump.
*/
void TrajectoryJump(float t, float launchTime, float stanceHeight,
    float downAMP, float& x, float& y) {
    //Need to check if n works
    float n = t/launchTime;
    x = 0;
    y = downAMP*n + stanceHeight;
    //y = downAMP*sin(PI/4 + PI/4*n) + stanceHeight;
}

/**
* Drives the ODrives in an open-loop, position-control sinTrajectory.
*/
void ExecuteJump() {
    Serial.println("Jump Initialized");

    // min radius = 0.8
    // max radius = 0.25
    const float launchTime = 5.0f ;
    const float stanceHeight = 0.05f; // Desired height of body from ground prior to jump (m)
    const float downAMP = 0.1f; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)

    float t = millis()/1000.0f - start_time_; // Seconds since jump was commanded
    if (t > launchTime) {
        execute_jump_ = false;
        Serial.println("Jump Complete");
    } else {
        float x,y;
        TrajectoryJump(t, launchTime, stanceHeight, downAMP, x, y);

        // Send commands to odrives
        float theta, gamma;
        CartesianToThetaGamma(x, y, 1.0, theta, gamma);
        struct LegGain gains = {120, 0.48, 120, 0.48};
        odrv0Interface.SetCoupledPosition(theta, gamma, gains);
        odrv1Interface.SetCoupledPosition(theta, gamma, gains);
        odrv2Interface.SetCoupledPosition(theta, gamma, gains);
        odrv3Interface.SetCoupledPosition(theta, gamma, gains);

        Serial << "Jump: +" << t << "s, y: " << y;
    }
}

#endif
