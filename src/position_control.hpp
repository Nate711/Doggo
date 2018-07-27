#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "config.hpp"
#include "globals.hpp"

#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

void CoupledPIDControl();
void ODrivePosControl();
void MotorAngleToLegAngles();

//------------------------------------------------------------------------------
// PositionControlThread: Motor position control thread
// Periodically calculates result from PID controller and sends off new motor
// current commands to the ODrive(s)

// TODO: add support for multiple ODrives

static THD_WORKING_AREA(waPositionControlThread, 128);

static THD_FUNCTION(PositionControlThread, arg) {
    (void)arg;

    while(true) {
        // CoupledPIDControl();
        ODrivePosControl();
    }
}

/**
 * All the conversions between the different coordinates are given below, we can move them to whatever file we want
 * This needs to define global variable for the motor angles, leg angles, and foot coordinates so the functions are void
 * We also have to define the upper and lower leg lengths L1 and L2 as global vars
 * Here the motor angles are assumed to be in radians with a 0 position in the x-direction along the body and positive angles CW
 * Make a MotorAngles struct with th1 and th2 values
 * The x,y cartesian coordinates have the x-direction pointed where the robot is heading and y-direction is positive towards the ground
 * Make struct with two floats for LegCartesian and give an x,y values
 * The leg length (L) is the virtual leg length and is always positive, the leg angle, gamma, is zero in the y-direction and positive when going CW
 * Make a struct with L,gamma values called LegParams
 */
float L1 = 0.15 // upper leg length (m) what is it actually?
float L2 = 0.3 // lower leg length (m)

/**
 * TODO: Need to change the th1, th2 values as defined above to the encoder counts to set for the position controller. Encoder counts per rev is 2000
 * Make this an if else and handle different orientations, offsets based on which motor is given in motorIndex
 */
int MotorAngleToEncoderCounts(theta, thetaIndex, motorIndex) {
  // have an if else based on if thetaIndex is 0 or 1 for either theta so just return 1 value and avoid matrix
}

/**
 * TODO: Need to change the encoder counts enc1, enc2 values to the motor angles as defined above
 * Make this an if else and handle different orientations, offsets based on which motor is given in motorIndex
 */
float EncoderCountsToMotorAngle(enc, encIndex, motorIndex) {
}

/**
 * Converts the two motor angles (in radians) for a leg to the cartesian coordinates
 */
void MotorAngleToCartesian(th1, th2) {
  delta = (th2+th1+PI)/2.0 - acos(L1/L2*sin((th2-th1)/2.0));
  x = L1*cos(th1) + L2*cos(delta);
  y = L1*sin(th1) + L2*sin(delta);
}

/**
 * Converts the two motor angles (in radians) for a leg into the motor angles th1, th2 (in rads)
 */
void LegParamsToMotorAngles(L, gamma) {
  beta = 2.0*acos((L1^2.0+L^2.0-L2^2.0)/(2.0*L1*L));
  th1 = (PI/2.0 + gamma - beta/2.0);
  th2 = (PI/2.0 + gamma + beta/2.0);
}

/**
 * Converts the leg params L, gamma to cartesian coordinates x, y (in m)
 */
void LegParamsToCartesian(L, gamma) {
  x = -L*sin(gamma);
  y = L*cos(gamma);
}

/**
 * Converts the cartesian coords x, y (m) to leg params L (m), gamma (rad)
 */
void CartesianToLegParams(x, y) {
  L = (x^2 + y^2)^0.5;
  gamma = atan(-x/y);
}

/**
 * Sinusoidal trajectory generator function with flexibility from parameters described below. Can do 4-beat, 2-beat, trotting, etc with this.
 */
void sinTrajectory(t, FREQ, gaitOffset, stanceHeight, flightPercent, stepLength, upAMP, downAMP) {
    gp = (FREQ*t+gaitOffset) % 1.0; // mod(a,m) returns remainder division of a by m
    if gp <= flightPercent{
      x = (gp/flightPercent)*stepLength - stepLength/2.0;
      y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;
    } else {
      percentBack = (gp-flightPercent)/(1.0-flightPercent);
      x = -percentBack*stepLength + stepLength/2.0;
      y = downAMP*sin(PI*percentBack) + stanceHeight;
    // TODO figure out if need to store x,y to global vars or return values in this function
    }
}

/**
 * Drives the ODrives in an open-loop, position-control sinTrajectory.
 */
void sinTrajectoryPosControl() {
    float stanceHeight = 0.2; // Desired height of body from ground during walking (m)
    float downAMP = 0.03; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    float upAMP = 0.1; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    float flightPercent = 0.25; // Portion of the gait time should be doing the down portion of trajectory
    float stepLength = 0.1; // Length of entire step (m)
    float FREQ = 2.0; // Frequency of one gait cycle (Hz)
    float gaitOffset1 = 0.0; // Phase shift in percent (i.e. 25% shift is 0.25) to be passed in depending on motor
    float t = millis()/1000.0;


    // Leg 0 // TODO figure out how we want the variables for the sinTrajectory
    const float leg0Offset = 0.0;
    l0coords = sinTrajectory(t, FREQ, leg0Offeset, stanceHeight, flightPercent, stepLength, upAMP, downAMP)
    float sp00 = MotorAngleToEncoderCounts(th00, 0, 0);
    float sp01 = MotorAngleToEncoderCounts(th01, 1, 0);

    // Leg 1
    const float leg1Offset = 0.25;
    sinTrajectory(t, FREQ, leg1Offeset, stanceHeight, flightPercent, stepLength, upAMP, downAMP)
    float sp10 = MotorAngleToEncoderCounts(th10, 0, 1);
    float sp11 = MotorAngleToEncoderCounts(th11, 1, 1);

    // Leg 2 // Nathan had sp20's and 30's switched?
    const float leg2Offset = 0.5;
    sinTrajectory(t, FREQ, leg2Offeset, stanceHeight, flightPercent, stepLength, upAMP, downAMP)
    float sp20 = MotorAngleToEncoderCounts(th20, 0, 2);
    float sp21 = MotorAngleToEncoderCounts(th21, 1, 2);

    // Leg 3
    const float leg3Offset = 0.75;
    sinTrajectory(t, FREQ, leg3Offeset, stanceHeight, flightPercent, stepLength, upAMP, downAMP)
    float sp30 = MotorAngleToEncoderCounts(th30, 0, 3);
    float sp31 = MotorAngleToEncoderCounts(th31, 1, 3);

    odrv0Interface.SetPosition(0,sp00);
    odrv0Interface.SetPosition(1,sp01);
    odrv1Interface.SetPosition(0,sp10);
    odrv1Interface.SetPosition(1,sp11);
    odrv2Interface.SetPosition(0,sp20);
    odrv2Interface.SetPosition(1,sp21);
    odrv3Interface.SetPosition(0,sp30);
    odrv3Interface.SetPosition(1,sp31);
}

/**
 * Drives the ODrives in an open-loop, position-control trajectory.
 */
void ODrivePosControl() {

    float FREQ = 1.0; // hz
    float AMP = 800.0;
    float t = millis()/1000.0 * FREQ;

    // Leg 0
    float sp00 = AMP*sin(2*PI*t);
    float sp01 = AMP*sin(2*PI*t + PI/2.0);

    // Leg 1
    float leg1_phase_shift = PI/2.0;
    float sp10 = AMP*sin(2*PI*t + leg1_phase_shift);
    float sp11 = AMP*sin(2*PI*t + leg1_phase_shift + PI/2.0);

    // Leg 2
    const float leg2_phase_shift = PI + PI;
    float sp30 = AMP*sin(2*PI*t + PI/2.0 + leg2_phase_shift);
    float sp31 = AMP*sin(2*PI*t + leg2_phase_shift);

    // Leg 3
    const float leg3_phase_shift = PI/2.0;
    float sp20 = AMP*sin(2*PI*t + PI/2.0 + leg3_phase_shift);
    float sp21 = AMP*sin(2*PI*t + leg3_phase_shift);

    odrv0Interface.SetPosition(0,sp00);
    odrv0Interface.SetPosition(1,sp01);
    odrv1Interface.SetPosition(0,sp10);
    odrv1Interface.SetPosition(1,sp11);
    odrv2Interface.SetPosition(0,sp20);
    odrv2Interface.SetPosition(1,sp21);
    odrv3Interface.SetPosition(0,sp30);
    odrv3Interface.SetPosition(1,sp31);

    chThdSleepMilliseconds(5);
}

/**
 * Computes PID onboard the teensy to enable coupled control on the legs.
 * Right now it only computes control for a single leg.
 */
void CoupledPIDControl() {
    float alpha = (float) odrv0.axis0.abs_pos_estimate;
    float beta = (float) odrv0.axis1.abs_pos_estimate;

    float theta = alpha/2.0 + beta/2.0;
    float gamma = beta/2.0 - alpha/2.0;

    float theta_sp = 0; // TODO take as struct or something
    float gamma_sp = 0; // TODO take as struct or something

    float p_term_theta = leg_default.Kp_theta * (theta_sp - theta);
    float d_term_theta = leg_default.Kd_theta * (0); // TODO: Add motor velocities to position message from odrive

    float p_term_gamma = leg_default.Kp_gamma * (gamma_sp - gamma);
    float d_term_gamma = leg_default.Kd_gamma * (0); // TODO: Add motor velocities to position message from odrive

    // TODO: clamp (ie constrain) the outputs to -1.0 to 1.0
    float tau_theta = p_term_theta + d_term_theta;
    float tau_gamma = p_term_gamma + d_term_gamma;

    // TODO: check signs
    float tau_alpha = tau_theta*0.5 - tau_gamma*0.5;
    float tau_beta = tau_theta*0.5 + tau_gamma*0.5;
    // odrv0Interface.SetDualCurrent(tau_alpha, tau_gamma);

    latest_send_timestamp = micros();

#ifdef DEBUG_HIGH
    Serial << "Sending at: " << micros() << '\n';
#endif
    // DEBUG only: send two zero current commands
    odrv0Interface.SetDualCurrent(0, 0);
#ifdef DEBUG_HIGH
    Serial << "Sent at: " << micros() << '\n';
#endif

#ifdef PRINT_ONCE
    chThdSleepMilliseconds(1000000);
#endif
    // The duration of sleep controls the loop frequency
    chThdSleepMicroseconds(1000000/POSITION_CONTROL_FREQ);
}

#endif
