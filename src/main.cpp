// This example illustrates cooperative scheduling. Cooperative scheduling
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
//
#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"

volatile uint32_t count = 0;
volatile uint32_t maxDelay = 0;

//------------------------------------------------------------------------------
// Initialize objects related to ODrives

// Make references to Teensy <-> computer serial (aka USB) and the ODrive(s)
HardwareSerial& odrv0Serial = Serial1;

// Make structs to hold motor readings
// TODO: figure out if I want to mimic the ODive struct style or not
struct ODrive {
    struct Axis {
        float pos_estimate = 0;
    };
    Axis axis0,axis1;
} odrv0;

// ODriveArduino objects
// These objects are responsible for sending commands to the ODrive
ODriveArduino odrv0Interface(odrv0Serial);

//------------------------------------------------------------------------------
// Helper utilities
// Add support for using "<<" to stream stuff to the usb serial
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//------------------------------------------------------------------------------
// E-STOP function
void ESTOP() {
    while(true) {}
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
// PrintDebugThread: Print debug information to computer at fixed rate
// TODO: characterize how much bandwidth it uses
static THD_WORKING_AREA(waPrintDebugThread, 256);
static THD_FUNCTION(PrintDebugThread, arg) {
    (void)arg;
    int count = 0;

    while(true) { // execute at 50hz
        if(count == 50) { // print variable name header every 50 prints (1s)
            Serial << "odrv0.axis0.pos_estimate\todrv0.axis1.pos_estimate\n";
            count = 0;
        }
        // Print odrv0 positions
        Serial << odrv0.axis0.pos_estimate << "\t" << odrv0.axis1.pos_estimate << "\n";

        count++;
        chThdSleepMilliseconds(20);
    }
}

//------------------------------------------------------------------------------
// PositionControlThread: Motor position control thread
// Periodically calculates result from PID controller and sends off a new
// dual current command to the ODrive(s)
// TODO: add support for multiple ODrives

static THD_WORKING_AREA(waPositionControlThread, 128);

static THD_FUNCTION(PositionControlThread, arg) {
    (void)arg;

    // DEBUG only: send two zero current commands
    // NOTE: when odrive is in closed loop position control I doubt
    // current commands will do anything
    odrv0Interface.SetDualCurrent(0.0, 0.0);

    chThdSleepMicroseconds(2000); // execute at 500Hz approximately
}

//------------------------------------------------------------------------------
// SerialThread: receive serial messages from ODrive.
// Pulls bytes from the odrv0 serial buffer (aka Serial1) at a rate of 100khz.
// When a newline char is received, it calls parseEncoderMessage to update the
// struct associated with the serial buffer.
// TODO: add timeout behavior: throw out buffer if certain time has elapsed since
// a new message has started being received

void parsePositionMsg(char* msg, int len);

// 128 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waSerialThread, 128);

static THD_FUNCTION(SerialThread, arg) {
    (void)arg;

    const int BUFFER_SIZE = 32;
    char msg[BUFFER_SIZE]; // running buffer of received characters
    int msg_idx = 0; // keep track of which index to write to

    while (true) {
        while(odrv0Serial.available()) {
            // reset buffer TODO deal with consequences of buffer overflow
            if(msg_idx >= BUFFER_SIZE) {
                Serial << "Msg buffer exceeded!\n";
                msg_idx = 0;
            }
            // Read latest byte out of the serial buffer
            char c = odrv0Serial.read();
            // Add the char to our buffer
            msg[msg_idx++] = c;

            // Check if we got stop character, aka newline
            if(c == '\n') {
                parsePositionMsg(msg,msg_idx);
                // TODO: add checksum
                msg_idx = 0;
            }
        }
        // Run the serial checking loop at 100khz by delaying 10us
        // chThdSleepMilliseconds(10);
        // TODO: make this interrupt driven?
        // Yielding here gives other threads a chance to execute
        chThdYield();
    }
}

/**
 * Parse a dual position message and store the result in the odrive struct
 * @param msg char* : message
 * @param len int   : message length
 * TODO: make it generalizable to other odrives and other odriveInterfaces
 */
void parsePositionMsg(char* msg, int len) {
    float m0,m1;
    int result = odrv0Interface.ParseDualPosition(msg, len, m0, m1);
    // result: 1 means success, -1 means didn't get proper message
    if (result == 1) {
        odrv0.axis0.pos_estimate = m0;
        odrv0.axis1.pos_estimate = m1;
    } else {
        // TODO: deal with message error
    }
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
    chThdCreateStatic(waIdleThread, sizeof(waIdleThread),
        NORMALPRIO, IdleThread, NULL);

    chThdCreateStatic(waPositionControlThread, sizeof(waPositionControlThread),
        NORMALPRIO, PositionControlThread, NULL);
    chThdCreateStatic(waSerialThread, sizeof(waSerialThread),
        NORMALPRIO, SerialThread, NULL);

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
