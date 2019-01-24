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

// ODriveArduino objects
// These objects are responsible for sending commands to the ODrive over their
// respective serial port
extern ODriveArduino odrv0Interface, odrv1Interface, odrv2Interface, odrv3Interface;

//------------------------------------------------------------------------------
// Global variables. These are needed for cross-thread communication!!

// Number of idle cycles per second
extern volatile uint32_t count;
// Maximum time between idle cycles
extern volatile uint32_t maxDelay;

// The last time (in microseconds) that the Teensy sent a message to an ODrive
extern volatile long latest_send_timestamp;
// The last time (in microseconds) that the Teensy received a message from an ODrive
extern volatile long latest_receive_timestamp;

// Make structs to hold motor readings and setpoints
struct ODrive {
    float sp_gamma = 0;
    float sp_theta = 0; // set point values
    float est_theta = 0;
    float est_gamma = 0; // actual values from the odrive
};

struct IMU {
    float yaw, pitch, roll; // Euler angles for robot body
};

// Struct to hold information helpful for debugging/printing to serial monitor
struct DebugValues {
    float t;
    long position_reply_time;
    struct ODrive odrv0, odrv1, odrv2, odrv3;
    struct IMU imu;
};

extern struct DebugValues global_debug_values;

extern bool enable_debug;

#endif
