#include "globals.h"
#include "Arduino.h"
#include "ODriveArduino.h"

//------------------------------------------------------------------------------
// Initialize objects related to ODrives

// TODO: There's a lot of repetition in this section that's hinting we should
// somehow encapsulate more behavior. We could put the serial references inside
// the ODriveArduino class and put the pos estimate struct in there too

// Make references to Teensy <-> computer serial (aka USB) and the ODrive(s)
HardwareSerial& odrv0Serial = Serial1;
HardwareSerial& odrv1Serial = Serial2;
HardwareSerial& odrv2Serial = Serial3;
HardwareSerial& odrv3Serial = Serial4;

// Make structs to hold motor readings
// TODO: figure out if I want to mimic the ODive struct style or not
struct ODrive odrv0, odrv1, odrv2, odrv3;

// ODriveArduino objects
// These objects are responsible for sending commands to the ODrive over their
// respective serial port
ODriveArduino odrv0Interface(odrv0Serial);
ODriveArduino odrv1Interface(odrv1Serial);
ODriveArduino odrv2Interface(odrv2Serial);
ODriveArduino odrv3Interface(odrv3Serial);

//------------------------------------------------------------------------------
// Global variables. These are needed for cross-thread communication!!

// Number of idle cycles per second
volatile uint32_t count = 0;
// Maximum time between idle cycles
volatile uint32_t maxDelay = 0;

// The last time (in microseconds) that the Teensy sent a message to an ODrive
volatile long latest_send_timestamp = 0;
// The last time (in microseconds) that the Teensy received a message from an ODrive
volatile long latest_receive_timestamp = 0;

// Struct to hold information helpful for debugging/printing to serial monitor
struct DebugValues global_debug_values;

bool enable_debug = false;
