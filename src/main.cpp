// Extreme Mobility Doggo Code

// Notes from cooperative scheduling ChibiOS example:
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
#include "globals.h"
#include "uart.h"
#include "position_control.h"
#include "usb_serial.h"
#include "debug.h"
#include "config.h"
#include "jump.h"
#include "datalog.h"
#include "imu.h"

//------------------------------------------------------------------------------
// E-STOP function
void ESTOP() {
    while(true) {}
}

//------------------------------------------------------------------------------
// IdleThread: Increments a counter so we know how many idle cycles we had
// per second. Also records the maximum time between running the idle thread, ie,
// the maximum time for which the cpu was doing stuff and didn't get back to the
// idle thread.

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
// BlinkThread: Blink the built-in led at 1Hz so you know if the Teensy is on.

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
// chSetup thread. Begins the program threads.
// Continue setup() after chBegin().
void chSetup() {
    // Checks to make sure you enabled cooperature scheduling
    if (CH_CFG_TIME_QUANTUM) {
        Serial.println("You must set CH_CFG_TIME_QUANTUM zero in");
        Serial.print("src/arm/chconf_arm.h");
        Serial.println(F(" to enable cooperative scheduling."));
        while (true) {}
    }

    // Create ALL the threads!!
    // This is the most important part of the setup

    // Idle thread: increments counter.
    chThdCreateStatic(waIdleThread, sizeof(waIdleThread),
        NORMALPRIO, IdleThread, NULL);

    // Control thread: executes PID and controls the motors.
    chThdCreateStatic(waPositionControlThread, sizeof(waPositionControlThread),
        NORMALPRIO, PositionControlThread, NULL);

    // Serial thread: reads any incoming serial messages from ODrives.
    chThdCreateStatic(waSerialThread, sizeof(waSerialThread),
        NORMALPRIO, SerialThread, NULL);

    // TODO: add sensor polling thread
    // TODO: create gait pattern thread (aka one that coordinates leg by generating leg setpoints)

    // USB Serial Thread: reads any incoming serial messages from the computer
    chThdCreateStatic(waUSBSerialThread, sizeof(waUSBSerialThread), NORMALPRIO,
        USBSerialThread, NULL);

    // Debug thread: prints out helpful debugging information to serial monitor
    chThdCreateStatic(waPrintDebugThread, sizeof(waPrintDebugThread),
        NORMALPRIO, PrintDebugThread, NULL);

    // Blink thread: blinks the onboard LED
    chThdCreateStatic(waBlinkThread, sizeof(waBlinkThread),
        NORMALPRIO, BlinkThread, NULL);

    // Datalog Thread: logs IMU data
    chThdCreateStatic(waDatalogThread, sizeof(waDatalogThread),
        NORMALPRIO, DatalogThread, NULL);

    // IMU Thread: Queries IMU and stores data
    chThdCreateStatic(waIMUThread, sizeof(waIMUThread),
        NORMALPRIO, IMUThread, NULL);
}
//------------------------------------------------------------------------------
// Setup thread.
// Responsible for initializing the serial ports
void setup() {
    #ifndef __arm__
    Serial.println("Must run on Teensy 3.5");
    while(true){}
    #endif

    // Begin at 115200 BAUD
    Serial.begin(115200);
    // Wait for USB Serial.
    while (!Serial) {}

    PrintStates();
    PrintGaitCommands();

    // Make sure the custom firmware is loaded because the default BAUD is 115200
    odrv0Serial.begin(500000);
    odrv1Serial.begin(500000);
    odrv2Serial.begin(500000);
    odrv3Serial.begin(500000);
    // TODO: figure out if i should wait for serial available... or some indication the odrive is on

    // Start ChibiOS.
    chBegin(chSetup);
    // chBegin() resets stacks and should never return.
    while (true) {} // TODO: what happens if you dont hold here?
}
//------------------------------------------------------------------------------
// Loop thread.
// Prints the number of idle cycles and maximum delay every 1 second.
void loop() {
    while (true) {
        // Serial << "Idle thd execs, max micros btn idle: \t";
        // Serial << count << "," << maxDelay << "\n";
        count = 0;
        maxDelay = 0;

        // Allow other threads to run for 1 sec.
        chThdSleepMilliseconds(1000);
    }
}
