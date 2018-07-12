#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "config.hpp"

#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

void CoupledPIDControl();
void ODrivePosControl();

//------------------------------------------------------------------------------
// PositionControlThread: Motor position control thread
// Periodically calculates result from PID controller and sends off a new
// dual current command to the ODrive(s)

// TODO: add support for multiple ODrives

static THD_WORKING_AREA(waPositionControlThread, 128);

static THD_FUNCTION(PositionControlThread, arg) {
    (void)arg;

    while(true) {
        // CoupledPIDControl();
        ODrivePosControl();
    }
}

void ODrivePosControl() {
    odrv0Interface.SetPosition(0,200);

    float t = millis()/1000;
    float sp = 2000*sin(2*PI*t); // 1hz sinusoid
    odrv0Interface.SetPosition(1,sp);

    chThdSleepMilliseconds(10);
}

void CoupledPIDControl() {
    // PID loop FOR ONE LEG
    float alpha = (float) odrv0.axis0.abs_pos_estimate;
    float beta = (float) odrv0.axis1.abs_pos_estimate;

    float theta = alpha/2.0 + beta/2.0;
    float gamma = beta/2.0 - alpha/2.0;

    float theta_sp = 0; // TODO take as struct or something
    float gamma_sp = 0; // TODO take as struct or something

    float p_term_theta = leg0.Kp_theta * (theta_sp - theta);
    float d_term_theta = leg0.Kd_theta * (0); // TODO: Add motor velocities to position message from odrive

    float p_term_gamma = leg0.Kp_gamma * (gamma_sp - gamma);
    float d_term_gamma = leg0.Kd_gamma * (0); // TODO: Add motor velocities to position message from odrive

    // TODO: clamp (ie constrain) the outputs to -1.0 to 1.0
    float tau_theta = p_term_theta + d_term_theta;
    float tau_gamma = p_term_gamma + d_term_gamma;

    // TODO: check signs
    float tau_alpha = tau_theta*0.5 - tau_gamma*0.5;
    float tau_beta = tau_theta*0.5 + tau_gamma*0.5;
    // odrv0Interface.SetDualCurrent(tau_alpha, tau_gamma);

    latest_send_timestamp = micros();

    // DEBUG only: send two zero current commands
    odrv0Interface.SetDualCurrent(0, 0);

    // The duration of sleep controls the loop frequency
    chThdSleepMicroseconds(1000000/POSITION_CONTROL_FREQ);
}

#endif
