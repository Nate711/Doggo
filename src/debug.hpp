#include "ChRt.h"
#include "Arduino.h"
#include "config.hpp"


#ifndef DEBUG_H
#define DEBUG_H

//------------------------------------------------------------------------------
// PrintDebugThread: Print debugging information to the serial montior at fixed rate
// 
// TODO: characterize how much bandwidth it uses
static THD_WORKING_AREA(waPrintDebugThread, 256);
static THD_FUNCTION(PrintDebugThread, arg) {
    (void)arg;
    int count = 0;

    while(true) { // execute at 10hz
        // Print a line saying the variable names every 1s
        // if(count == DEBUG_PRINT_FREQ) {
        //     Serial << "odrv0.axis0.pos_estimate\todrv0.axis1.pos_estimate\n";
        //     count = 0;
        // }

        // Print odrv0 positions
        Serial  << "M0, M1, feedback time: " ;
        Serial  << global_debug_values.odrv0.axis0.pos_estimate ;
        Serial  << "\t" ;
        Serial  << global_debug_values.odrv0.axis1.pos_estimate ;
        Serial  << "\t" ;
        Serial  << global_debug_values.feedback_loop_time;
        Serial  << "\n";

        count++;
        chThdSleepMilliseconds(1000/DEBUG_PRINT_FREQ);
    }
}

#endif
