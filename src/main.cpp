// Extreme Mobility Doggo Code

// Notes from cooperative scheduling ChibiOS example
// This code uses cooperative scheduling. Cooperative scheduling
// simplifies multitasking since no preemptive context switches occur.
//
// You must call chYield() or other ChibiOS functions such as chThdSleep
// to force a context switch to other threads.
//
// Insert a delay(100) in loop() to see the effect of not
// using chThdSleep with cooperative scheduling.
//
// Setting CH_TIME_QUANTUM to zero disables the preemption for threads
// with equal priority and the round robin becomes cooperative.
// Note that higher priority threads can still preempt, the kernel
// is always preemptive.

#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "globals.hpp"
#include "uart.hpp"
#include "position_control.hpp"
#include "debug.hpp"
#include "config.hpp"

//------------------------------------------------------------------------------
// E-STOP function
void ESTOP() {
    while(true) {}
}

//------------------------------------------------------------------------------
// IdleThread: increment a counter and records max delay.
// The max delay is the maximum time chThdYield takes up. chThdYield tells the
// OS to run other threads.
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waIdleThread, 64);

static THD_FUNCTION(IdleThread, arg) {
    (void)arg;
    while (true) {
        count++;
        uint32_t t = micros();
        // Yield so other threads can run.
        chThdYield();
        t = micros() - t;
        if (t > maxDelay) maxDelay = t;
    }
}

//------------------------------------------------------------------------------
// BlinkThread: Blink the built-in led
// 64 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waBlinkThread, 64);

static THD_FUNCTION(BlinkThread, arg) {
    (void)arg;
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH);
        chThdSleepMilliseconds(500);
        digitalWrite(LED_BUILTIN, LOW);
        chThdSleepMilliseconds(500);
    }
}


//------------------------------------------------------------------------------
// Continue setup() after chBegin().
void chSetup() {
    // Checks to make sure you enabled cooperature scheduling
    if (CH_CFG_TIME_QUANTUM) {
        Serial.println("You must set CH_CFG_TIME_QUANTUM zero in");
        Serial.print("src/arm/chconfig_arm.h");
        Serial.println(F(" to enable cooperative scheduling."));
        while (true) {}
    }

    // Create ALL the threads!!
    // This is the most important part of the setup

    // Idle thread
    chThdCreateStatic(waIdleThread, sizeof(waIdleThread),
        NORMALPRIO, IdleThread, NULL);

    // Control threads
    chThdCreateStatic(waPositionControlThread, sizeof(waPositionControlThread),
        NORMALPRIO, PositionControlThread, NULL);

    chThdCreateStatic(waSerialThread, sizeof(waSerialThread),
        NORMALPRIO, SerialThread, NULL);

    // TODO: add sensor polling thread
    // TODO: create gait pattern thread (aka one that coordinates leg by generating leg setpoints)

    // Debug threads
    chThdCreateStatic(waPrintDebugThread, sizeof(waPrintDebugThread),
        NORMALPRIO, PrintDebugThread, NULL);

    chThdCreateStatic(waBlinkThread, sizeof(waBlinkThread),
        NORMALPRIO, BlinkThread, NULL);
}
//------------------------------------------------------------------------------
void setup() {
    #ifndef __arm__
    Serial.println("Must run on Teensy 3.5");
    while(true){}
    #endif

    Serial.begin(115200);
    // Wait for USB Serial.
    while (!Serial) {}

    // Make sure the custom firmware is loaded because the default BAUD is 115200
    odrv0Serial.begin(500000);
    // TODO: figure out if i should wait for serial available... or some indication the odrive is on

    // Start ChibiOS.
    chBegin(chSetup);
    // chBegin() resets stacks and should never return.
    while (true) {} // TODO: what happens if you dont hold here?
}
//------------------------------------------------------------------------------
void loop() {
    while (true) {
        Serial << "Idle thd execs, max micros btn idle: \t";
        Serial << count << "," << maxDelay << "\n";
        count = 0;
        maxDelay = 0;

        // Allow other threads to run for 1 sec.
        chThdSleepMilliseconds(1000);
    }
}
