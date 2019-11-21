#include "debug.h"
#include "ChRt.h"
#include "Arduino.h"
#include "config.h"
#include "globals.h"
#include "haptics.h"


static float deadband(float x, float db) {
    return abs(x) > db ? x : 0;
}

//------------------------------------------------------------------------------
// PrintDebugThread: Print debugging information to the serial montior at fixed rate
//
// TODO: characterize how much bandwidth it uses
THD_WORKING_AREA(waPrintDebugThread, 1024);
THD_FUNCTION(PrintDebugThread, arg) {
    (void)arg;
    float events[13];
    ODrive * odrvs[4] = {&global_debug_values.odrv0, &global_debug_values.odrv1, 
                       &global_debug_values.odrv2, &global_debug_values.odrv3};
    initializeHaptics();
    while(true) { // execute at 10hz
        // Print a line saying the variable names every 1s
        // if(count == DEBUG_PRINT_FREQ) {
        //     Serial << "odrv0.axis0.pos_estimate\todrv0.axis1.pos_estimate\n";
        //     count = 0;
        // }
        getAllEvents(odrvs, events);
        getEvent(10, global_debug_values.imu.accelX, events);
        getEvent(11, global_debug_values.imu.accelY, events);
        getEvent(12, global_debug_values.imu.accelZ, events);
        for (int i = 0; i < 4; i++) {
                odrvs[i]->current_0 = NAN;
        }
        odrv0Interface.ReadCurrents();
        odrv1Interface.ReadCurrents();
        odrv2Interface.ReadCurrents();
        odrv3Interface.ReadCurrents();
        processEvents(events);
        if (enable_debug) {
            Serial.print(deadband(events[10], 1.5));
            Serial.print(",");
            Serial.print(deadband(events[11], 1.5));
            Serial.print(",");
            Serial.print(deadband(events[12], 1.5));
            Serial.println();
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