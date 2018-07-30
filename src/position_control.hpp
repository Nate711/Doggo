#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "config.hpp"
#include "globals.hpp"

#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

void CoupledPIDControl();
void ODrivePosControl();
void HipAngleToEncoderCounts(float alpha, float beta, float& enc1, float& enc2);
void EncoderCountsToHipAngle(float enc1, float enc2, float& alpha, float& beta);
void HipAngleToCartesian(float alpha, float beta, float& x, float& y);
void LegParamsToHipAngles(float L, float theta, float& alpha, float& beta);
void LegParamsToCartesian(float L, float theta, float& x, float& y);
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta);
void sinTrajectory(float t, float FREQ, float gaitOffset, float stanceHeight, float flightPercent, float stepLength, float upAMP, float downAMP, float& x, float& y);
void CartesianToEncoder(float x, float y, float leg_direction, float sign, float& enc0, float& enc1);
void MoveLeg(ODriveArduino& odrive, float t, float FREQ, float gait_offset, float stanceHeight, float flightPercent, float stepLength, float upAMP, float downAMP, float leg_direction, float sign);
void sinTrajectoryPosControl();
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
        // ODrivePosControl();
        sinTrajectoryPosControl();
    }
}

/**
* All the conversions between the different coordinates are given below, we can move them to whatever file we want
* This needs to define global variable for the Hip angles, leg angles, and foot coordinates so the functions are void
* We also have to define the upper and lower leg lengths L1 and L2 as global vars
* Here the Hip angles are assumed to be in radians with a 0 position in the x-direction along the body and positive angles CW
* Make a HipAngles struct with alpha and beta values
* The x,y cartesian coordinates have the x-direction pointed where the robot is heading and y-direction is positive towards the ground
* Make struct with two floats for LegCartesian and give an x,y values
* The leg length (L) is the virtual leg length and is always positive, the leg angle, theta, is zero in the x-direction and positive when going CW
* Make a struct with L,theta values called LegParams
*/

/**
* Make initial angles for alpha and beta 0 and PI before turning on so that the encoder offset is correctly initialzed.
* Sign is just 1.0 or -1.0 if the sign of the encoders needs to be flipped
* Need to change the alpha, beta values as defined above to the encoder counts to set for the position controller. Encoder counts per rev is 2000
* Make this an if else and handle different orientations, offsets based on which Hip is given in HipIndex
*/
void HipAngleToEncoderCounts(float alpha, float beta, float sign, float& enc0, float& enc1) {
    // have an if else based on if thetaIndex is 0 or 1 for either theta so just return 1 value and avoid matrix
    const float enc_offset = -PI; // based on initialization
    const float gear_ratio = 3.0;
    enc0 = sign * alpha/ (2*PI) * 2000.0 * gear_ratio;
    enc1 = sign * (beta + enc_offset) / (2*PI) * 2000.0 * gear_ratio;
}

/**
* TODO: Need to change the encoder counts enc1, enc2 values to the Hip angles as defined above
* Make this an if else and handle different orientations, offsets based on which Hip is given in HipIndex
*/
void EncoderCountsToHipAngle(float enc1, float enc2, float& alpha, float& beta) {

}

/**
* Converts the two Hip angles (in radians) for a leg to the cartesian coordinates
*/
void HipAngleToCartesian(float alpha, float beta, float& x, float& y) {
    float L1 = 0.09; // upper leg length (m) what is it actually?
    float L2 = 0.162; // lower leg length (m)
    float delta = (beta+alpha+PI)/2.0 - acos(L1/L2*sin((beta-alpha)/2.0));
    x = L1*cos(alpha) + L2*cos(delta);
    y = L1*sin(alpha) + L2*sin(delta);
}

/**
* Converts the two Hip angles (in radians) for a leg into the Hip angles alpha, beta (in rads)
*/
void LegParamsToHipAngles(float L, float theta, float& alpha, float& beta) {
    float L1 = 0.09; // upper leg length (m) what is it actually?
    float L2 = 0.162; // lower leg length (m)
    float cos_param = (pow(L1,2.0) + pow(L,2.0) - pow(L2,2.0)) / (2.0*L1*L);
    float gamma;
    if (cos_param < -1.0) {
        gamma = PI;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too small to find valid alpha and beta!");
        #endif
    } else if (cos_param > 1.0) {
        gamma = 0;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too large to find valid alpha and beta!");
        #endif
    } else {
        gamma = acos(cos_param);
    }
    alpha = theta - gamma;
    beta = theta + gamma;
}

/**
* Converts the leg params L, gamma to cartesian coordinates x, y (in m)
* Set x_direction to 1.0 or -1.0 to change which direction the leg walks
*/
void LegParamsToCartesian(float L, float theta, float leg_direction, float& x, float& y) {
    x = leg_direction * L * cos(theta);
    y = L * sin(theta);
}

/**
* Converts the cartesian coords x, y (m) to leg params L (m), gamma (rad)
*/
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta) {
    L = pow( (pow(x,2.0) + pow(y,2.0)) ,0.5);
    theta = atan2(y,leg_direction * x);
}

/**
* Sinusoidal trajectory generator function with flexibility from parameters described below. Can do 4-beat, 2-beat, trotting, etc with this.
*/
void sinTrajectory (float t, float FREQ, float gaitOffset, float stanceHeight,
                    float flightPercent, float stepLength, float upAMP,
                    float downAMP, float& x, float& y) {
    float gp = fmod((FREQ*t+gaitOffset),1.0); // mod(a,m) returns remainder division of a by m
    if (gp <= flightPercent) {
        x = (gp/flightPercent)*stepLength - stepLength/2.0;
        y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;
    }
    else {
        float percentBack = (gp-flightPercent)/(1.0-flightPercent);
        x = -percentBack*stepLength + stepLength/2.0;
        y = downAMP*sin(PI*percentBack) + stanceHeight;
    }
}

void CartesianToEncoder(float x, float y, float leg_direction, float enc_sign,
                        float& enc0, float& enc1){
    float L = 0.0;
    float theta = 0.0;
    CartesianToLegParams(x, y, leg_direction, L, theta);
    float alpha = 0.0;
    float beta = 0.0;
    LegParamsToHipAngles(L, theta, alpha, beta);
    HipAngleToEncoderCounts(alpha, beta, enc_sign, enc0, enc1);

    Serial.print(x,4);
    Serial.print(" ");
    Serial.print(y,4);
    Serial.print(" ");
    Serial.print(L);
    Serial.print(" ");
    Serial.print(theta);
    Serial.println();
}

void MoveLeg(ODriveArduino& odrive, float t, float FREQ, float gait_offset,
             float stanceHeight, float flightPercent, float stepLength,
             float upAMP, float downAMP, float leg_direction, float sign) {
    float enc0;
    float enc1;
    float x; // float x for leg 0 to be set by the sin trajectory
    float y;
    sinTrajectory(t, FREQ, gait_offset, stanceHeight, flightPercent, stepLength, upAMP, downAMP, x, y);
    CartesianToEncoder(x, y, leg_direction, sign, enc0, enc1);
    odrive.SetPosition(0,(int)enc0);
    odrive.SetPosition(1,(int)enc1);
}

/**
* Drives the ODrives in an open-loop, position-control sinTrajectory.
*/
void sinTrajectoryPosControl() {
    // min radius = 0.8
    // max radius = 0.25
    const float stanceHeight = 0.15; // Desired height of body from ground during walking (m)
    const float downAMP = 0.01; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    const float upAMP = 0.07; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    const float flightPercent = 0.10; // Portion of the gait time should be doing the down portion of trajectory
    const float stepLength = 0.08; // Length of entire step (m)
    const float FREQ = 0.6; // Frequency of one gait cycle (Hz)
    const float gaitOffset1 = 0.0; // Phase shift in percent (i.e. 25% shift is 0.25) to be passed in depending on Hip
    float t = millis()/1000.0;

    const float leg0_offset = 0.0;
    const float leg0_sign = -1.0;
    const float leg0_direction = -1.0;
    MoveLeg(odrv0Interface, t, FREQ, leg0_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg0_direction, leg0_sign);

    const float leg1_offset = 0.25;
    const float leg1_sign = -1.0;
    const float leg1_direction = -1.0;
    MoveLeg(odrv1Interface, t, FREQ, leg1_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg1_direction, leg1_sign);

    const float leg2_offset = 0.75;
    const float leg2_sign = -1.0;
    const float leg2_direction = 1.0;
    MoveLeg(odrv2Interface, t, FREQ, leg2_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg2_direction, leg2_sign);

    const float leg3_offset = 0.5;
    const float leg3_sign = -1.0;
    const float leg3_direction = 1.0;
    MoveLeg(odrv3Interface, t, FREQ, leg3_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg3_direction, leg3_sign);
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
