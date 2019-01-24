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
            PrintODriveDebugInfo(global_debug_values.odrv0);
            Serial << '\t';
            PrintODriveDebugInfo(global_debug_values.odrv1);
            Serial << '\t';
            PrintODriveDebugInfo(global_debug_values.odrv2);
            Serial << '\t';
            PrintODriveDebugInfo(global_debug_values.odrv3);
            Serial << '\t';
            Serial << global_debug_values.imu.pitch;
            Serial.println();
        }

        chThdSleepMilliseconds(1000/DEBUG_PRINT_FREQ);
    }
}

void PrintODriveDebugInfo(struct ODrive odrv) {
    Serial.print(odrv.sp_theta, 2);
    Serial.print("\t");
    Serial.print(odrv.est_theta, 2);
    Serial.print("\t");
    Serial.print(odrv.sp_gamma, 2);
    Serial.print("\t");
    Serial.print(odrv.est_gamma, 2);
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
