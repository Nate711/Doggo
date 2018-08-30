#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "config.hpp"
#include "globals.hpp"

#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

void CoupledPIDControl();
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
        // sinTrajectoryPosControl();
        //sinTrajectoryPosControl();
        chThdSleepMilliseconds(10);

    }
}

/**
* All the conversions between the different coordinates are given below, we can move them to whatever file we want.
* This needs to define global variable for the Hip angles, leg angles, and foot coordinates so the functions are void.
* We also have to define the upper and lower leg lengths L1 and L2 as global vars
* Here the Hip angles are assumed to be in radians with a 0 position in the
* downward y-direction and positive angles CCW.
* TODO: Make a HipAngles struct with alpha and beta values
* The x,y cartesian coordinates have the x-direction pointed where the robot is
* heading and y-direction is positive towards the ground
* TODO: Make struct with two floats for LegCartesian and give an x,y values
* The leg length (L) is the virtual leg length and is always positive,
* the leg angle, theta, is zero in the downward y-direction and positive when
* going CWW.
* TODO: Make a struct with L,theta values called LegParams
*/


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
* Takes the leg parameters and returns the gamma angle (rad) of the legs
*/
void GetGamma(float L, float theta, float& gamma) {
    float L1 = 0.09; // upper leg length (m) what is it actually?
    float L2 = 0.162; // lower leg length (m)
    float cos_param = (pow(L1,2.0) + pow(L,2.0) - pow(L2,2.0)) / (2.0*L1*L);
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
}

/**
* Converts the two Hip angles (in radians) for a leg into the Hip angles alpha, beta (in rads)
*/
void LegParamsToHipAngles(float L, float theta, float& alpha, float& beta) {
    float gamma;
    GetGamma(L, theta, gamma);
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
* Converts the cartesian coords x, y (m) to leg params L (m), theta (rad)
*/
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta) {
    L = pow( (pow(x,2.0) + pow(y,2.0)) ,0.5);
    theta = atan2(leg_direction * x, y);
}

/**
* Sinusoidal trajectory generator function with flexibility from parameters described below. Can do 4-beat, 2-beat, trotting, etc with this.
*/
void SinTrajectory (float t, float FREQ, float gaitOffset, float stanceHeight,
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

void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma) {
    float L = 0.0;
    CartesianToLegParams(x, y, leg_direction, L, theta);
    GetGamma(L, theta, gamma);
    Serial.print(theta);
    Serial.print(" ");
    Serial.print(gamma);
    Serial.print('\n');
}

void CoupledMoveLeg(ODriveArduino& odrive, float t, float FREQ, float gait_offset,
             float stanceHeight, float flightPercent, float stepLength,
             float upAMP, float downAMP, float leg_direction) {
    float theta;
    float gamma;
    float x; // float x for leg 0 to be set by the sin trajectory
    float y;
    SinTrajectory(t, FREQ, gait_offset, stanceHeight, flightPercent, stepLength, upAMP, downAMP, x, y);
    CartesianToThetaGamma(x, y, leg_direction, theta, gamma);
    odrive.SetCoupledPosition(theta, 38.2, 0.48, gamma, 20, 0.48);
}

/**
* Pronk gait parameters
*/
void pronk() {
    // min radius = 0.8
    // max radius = 0.25
    const float stanceHeight = 0.15; // Desired height of body from ground during walking (m)
    const float downAMP = 0.05; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    const float upAMP = 0.05; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    const float flightPercent = 0.4; // Portion of the gait time should be doing the down portion of trajectory
    const float stepLength = 0.0; //0.12; // Length of entire step (m)
    const float FREQ = 1.0; // Frequency of one gait cycle (Hz)
    float t = millis()/1000.0;

    Serial.print(t);
    Serial.print(" ");

    const float leg0_offset = 0.0;
    const float leg0_direction = -1.0;
    CoupledMoveLeg(odrv0Interface, t, FREQ, leg0_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg0_direction);

    const float leg1_offset = 0.0;
    const float leg1_direction = -1.0;
    CoupledMoveLeg(odrv1Interface, t, FREQ, leg1_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg1_direction);

    const float leg2_offset = 0.0;
    const float leg2_direction = 1.0;
    CoupledMoveLeg(odrv2Interface, t, FREQ, leg2_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg2_direction);

    const float leg3_offset = 0.0;
    const float leg3_direction = 1.0;
    CoupledMoveLeg(odrv3Interface, t, FREQ, leg3_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg3_direction);
    Serial.println();
}


/**
* Trot gait parameters
*/
void trot() {
    // min radius = 0.8
    // max radius = 0.25
    const float stanceHeight = 0.15; // Desired height of body from ground during walking (m)
    const float downAMP = 0.0; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    const float upAMP = 0.05; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    const float flightPercent = 0.35; // Portion of the gait time should be doing the down portion of trajectory
    const float stepLength = 0.0; // Length of entire step (m)
    const float FREQ = 1.0; // Frequency of one gait cycle (Hz)
    float t = millis()/1000.0;

    Serial.print(t);
    Serial.print(" ");

    const float leg0_offset = 0.0;
    const float leg0_direction = -1.0;
    CoupledMoveLeg(odrv0Interface, t, FREQ, leg0_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg0_direction);

    const float leg1_offset = 0.5;
    const float leg1_direction = -1.0;
    CoupledMoveLeg(odrv1Interface, t, FREQ, leg1_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg1_direction);

    const float leg2_offset = 0.5;
    const float leg2_direction = 1.0;
    CoupledMoveLeg(odrv2Interface, t, FREQ, leg2_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg2_direction);

    const float leg3_offset = 0.0;
    const float leg3_direction = 1.0;
    CoupledMoveLeg(odrv3Interface, t, FREQ, leg3_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg3_direction);
    Serial.println();
}

/**
* Bound gait parameters
*/
void bound() {
    // min radius = 0.8
    // max radius = 0.25
    const float stanceHeight = 0.15; // Desired height of body from ground during walking (m)
    const float downAMP = 0.4; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
    const float upAMP = 0.04; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
    const float flightPercent = 0.5; // Portion of the gait time should be doing the down portion of trajectory
    const float stepLength = 0.0; // Length of entire step (m)
    const float FREQ = 1.0; // Frequency of one gait cycle (Hz)
    float t = millis()/1000.0;

    Serial.print(t);
    Serial.print(" ");

    const float leg0_offset = 0.0;
    const float leg0_direction = -1.0;
    CoupledMoveLeg(odrv0Interface, t, FREQ, leg0_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg0_direction);

    const float leg1_offset = 0.5;
    const float leg1_direction = -1.0;
    CoupledMoveLeg(odrv1Interface, t, FREQ, leg1_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg1_direction);

    const float leg2_offset = 0.0;
    const float leg2_direction = 1.0;
    CoupledMoveLeg(odrv2Interface, t, FREQ, leg2_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg2_direction);

    const float leg3_offset = 0.5;
    const float leg3_direction = 1.0;
    CoupledMoveLeg(odrv3Interface, t, FREQ, leg3_offset, stanceHeight,
        flightPercent, stepLength, upAMP, downAMP,
        leg3_direction);
    Serial.println();
}

#endif
