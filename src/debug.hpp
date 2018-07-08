#include "ChRt.h"
#include "Arduino.h"

#ifndef DEBUG_H
#define DEBUG_H

//------------------------------------------------------------------------------
// PrintDebugThread: Print debug information to computer at fixed rate
// TODO: characterize how much bandwidth it uses
static THD_WORKING_AREA(waPrintDebugThread, 256);
static THD_FUNCTION(PrintDebugThread, arg) {
    (void)arg;
    int count = 0;
    const int FREQ = 10;

    while(true) { // execute at 10hz
        if(count == FREQ) { // print variable name header every 1s
            Serial << "odrv0.axis0.pos_estimate\todrv0.axis1.pos_estimate\n";
            count = 0;
        }
        // Print odrv0 positions
        Serial << odrv0.axis0.pos_estimate << "\t" << odrv0.axis1.pos_estimate << "\n";

        count++;
        chThdSleepMilliseconds(1000/FREQ);
    }
}

#endif
