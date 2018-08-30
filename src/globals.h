#ifndef GLOBALS_H
#define GLOBALS_H

#include "ODriveArduino.h"

//------------------------------------------------------------------------------
// Helper utilities
// Add support for using "<<" to stream stuff to the usb serial
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//------------------------------------------------------------------------------
// Initialize objects related to ODrives

// TODO: There's a lot of repetition in this section that's hinting we should
// somehow encapsulate more behavior. We could put the serial references inside
// the ODriveArduino class and put the pos estimate struct in there too

// Make references to Teensy <-> computer serial (aka USB) and the ODrive(s)
extern HardwareSerial& odrv0Serial;
extern HardwareSerial& odrv1Serial;
extern HardwareSerial& odrv2Serial;
extern HardwareSerial& odrv3Serial;

// Make structs to hold motor readings
// TODO: figure out if I want to mimic the ODive struct style or not
struct ODrive {
    struct Axis {
        float pos_estimate = 0; // in counts
        float ENCODER_OFFSET = 0; // in counts, TODO: need to configure this

        // NOTE: abs_pos is the SUM of estiamte and offset
        float abs_pos_estimate = pos_estimate + ENCODER_OFFSET;
    };
    Axis axis0,axis1;
};

extern struct ODrive odrv0, odrv1, odrv2, odrv3;

// ODriveArduino objects
// These objects are responsible for sending commands to the ODrive over their
// respective serial port
extern ODriveArduino odrv0Interface, odrv1Interface, odrv2Interface, odrv3Interface;

//------------------------------------------------------------------------------
// Global variables. These are needed for cross-thread communication!!

// Struct to hold PID gains for the legs
struct LegGain {
    float Kp_theta = 0;
    float Kd_theta = 0;

    float Kp_gamma = 0;
    float Kd_gamma = 0;
};

extern struct LegGain leg_default;

// Number of idle cycles per second
extern volatile uint32_t count;
// Maximum time between idle cycles
extern volatile uint32_t maxDelay;

// The last time (in microseconds) that the Teensy sent a message to an ODrive
extern volatile long latest_send_timestamp;
// The last time (in microseconds) that the Teensy received a message from an ODrive
extern volatile long latest_receive_timestamp;

// Struct to hold information helpful for debugging/printing to serial monitor
struct DebugValues {
    long feedback_loop_time = 0;
    ODrive& odrv0 = odrv0;
    ODrive& odrv1 = odrv1;
    ODrive& odrv2 = odrv2;
    ODrive& odrv3 = odrv3;
};

extern struct DebugValues global_debug_values;

#endif
