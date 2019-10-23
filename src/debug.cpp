#include "debug.h"
#include "ChRt.h"
#include "Arduino.h"
#include "config.h"
#include "globals.h"

//------------------------------------------------------------------------------
// PrintDebugThread: Print debugging information to the serial montior at fixed rate
//
// TODO: characterize how much bandwidth it uses
THD_WORKING_AREA(waPrintDebugThread, 1024);
THD_FUNCTION(PrintDebugThread, arg) {
    (void)arg;

    while(true) { // execute at 10hz
        // Print a line saying the variable names every 1s
        // if(count == DEBUG_PRINT_FREQ) {
        //     Serial << "odrv0.axis0.pos_estimate\todrv0.axis1.pos_estimate\n";
        //     count = 0;
        // }

        if (enable_debug) {
            // Print leg positions
            // PrintODriveDebugInfo(global_debug_values.odrv0);
            // Serial << '\t';
            // PrintODriveDebugInfo(global_debug_values.odrv1);
            // Serial << '\t';
            // PrintODriveDebugInfo(global_debug_values.odrv2);
            // Serial << '\t';
            // PrintODriveDebugInfo(global_debug_values.odrv3);
            // Serial << '\t';
            // Serial << global_debug_values.imu.pitch;
            // Serial.println();
            PrintForces(global_debug_values.odrv0, global_debug_values.odrv1, global_debug_values.odrv2, global_debug_values.odrv3);

            global_debug_values.odrv0.current_0 = NAN;
            global_debug_values.odrv1.current_0 = NAN;
            global_debug_values.odrv2.current_0 = NAN;
            global_debug_values.odrv3.current_0 = NAN;
            odrv0Interface.ReadCurrents();
            odrv1Interface.ReadCurrents();
            odrv2Interface.ReadCurrents();
            odrv3Interface.ReadCurrents();
        }

        chThdSleepMilliseconds(1000/DEBUG_PRINT_FREQ);
    }
}

void PrintODriveDebugInfo(struct ODrive odrv) {
    // Serial.print(odrv.sp_theta, 2);
    // Serial.print("\t");
    Serial.print(odrv.est_theta, 2);
    Serial.print("\t");
    // Serial.print(odrv.sp_gamma, 2);
    // Serial.print("\t");
    Serial.print(odrv.est_gamma, 2);
    Serial.print("\t");
    Serial.print(odrv.current_0, 2);
    Serial.print("\t");
    Serial.print(odrv.current_1, 2);
    // Serial.printf("odrv%d: sp_th %.2f est_th %.2f sp_ga %.2f est_ga %.2f",
    //               odrvNum, odrv.sp_theta, 0.0,//odrv.est_theta,
    //               odrv.sp_gamma, 0.0);//odrv.est_gamma);
    // Serial.print()
    // Serial.print(odrv.sp_theta, 2);
    // Serial.print('\t');
    // Serial.print(odrv.est_theta, 2);
    // Serial.print('\t');
    // Serial.print(odrv.sp_gamma, 2);
    // Serial.print('\t');
    // Serial.print(odrv.est_gamma, 2);
}

static void invert(float mat[2][2]) {
    float a = mat[0][0];
    float b = mat[0][1];
    float c = mat[1][0];
    float d = mat[1][1];
    float invDet = 1.0 / (a * d - b * c);
    mat[0][0] = d * invDet;
    mat[0][1] = -b * invDet;
    mat[1][0] = -c * invDet;
    mat[1][1] = a * invDet;
}

static void calculateForce(struct ODrive odrv, float direction, float* xForce, float* yForce) {
    float gamma = odrv.est_gamma;
    float theta = odrv.est_theta;

    float L1 = 0.09F;
    float L2 = 0.162F;
    float k = 0.0245F * 3;

    float L = L1*cos(gamma) + sqrt(L2*L2 - L1*L1*sin(gamma)*sin(gamma));
    float dtheta_dm0 = -0.5F;
    float dtheta_dm1 = -0.5F;
    float dgamma_dm0 = -0.5F;
    float dgamma_dm1 = 0.5F;
    float dradius_dgamma = -L1*sin(gamma) - (L1*L1*sin(gamma)*cos(gamma))/(sqrt(L2*L2 - L1*L1*sin(gamma)*sin(gamma)));
    float dx_dtheta = L*cos(theta);
    float dy_dtheta = -L*sin(theta);
    float dx_dradius = sin(theta);
    float dx_dgamma = dx_dradius * dradius_dgamma;
    float dy_dradius = cos(theta);
    float dy_dgamma = dy_dradius * dradius_dgamma;

    float J[2][2] = {
        {dx_dtheta*dtheta_dm0+dx_dgamma*dgamma_dm0, dx_dtheta*dtheta_dm1+dx_dgamma*dgamma_dm1},
        {dy_dtheta*dtheta_dm0+dy_dgamma*dgamma_dm0, dy_dtheta*dtheta_dm1+dy_dgamma*dgamma_dm1}
    };
    float JT[2][2] = {
        {J[0][0], J[1][0]},
        {J[0][1], J[1][1]}
    };
    invert(JT);
    float tau0 = odrv.current_0 * k * direction;
    float tau1 = odrv.current_1 * k * direction;
    *xForce = JT[0][0] * tau0 + JT[0][1] * tau1;
    *yForce = JT[1][0] * tau0 + JT[1][1] * tau1;
}

void PrintForces(struct ODrive odrv0, struct ODrive odrv1, struct ODrive odrv2, struct ODrive odrv3) {
    float xForces = 0.0;
    float yForces = 0.0;
    float tempX, tempY;
    calculateForce(odrv0, -1.0, &tempX, &tempY);
    xForces -= tempX;
    yForces += tempY;
    calculateForce(odrv1, -1.0, &tempX, &tempY);
    xForces -= tempX;
    yForces += tempY;
    calculateForce(odrv2, 1.0, &tempX, &tempY);
    xForces += tempX;
    yForces += tempY;
    calculateForce(odrv3, 1.0, &tempX, &tempY);
    xForces += tempX;
    yForces += tempY;
    Serial.print(xForces);
    Serial.print(",");
    Serial.print(yForces);
    Serial.println();
}
