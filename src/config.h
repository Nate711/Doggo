#ifndef CONFIG_H
#define CONFIG_H

// Define DEBUG_LOW and/or DEBUG_HIGH to enable printing debug messages
// I've made two flags so you have more options in terms of which messages to show
// #define DEBUG_HIGH
// #define DEBUG_LOW
// #define PRINT_ONCE

//------------------------------------------------------------------------------
// Thread execution rates
#define POSITION_CONTROL_FREQ 100
#define DEBUG_PRINT_FREQ 20
#define UART_FREQ 2000
#define USB_SERIAL_FREQ 100
#define DATALOG_FREQ 10
#define IMU_FREQ 400
#define IMU_SEND_FREQ 100

//------------------------------------------------------------------------------
// Robot Safety Parameters
#define CURRENT_LIM 50.0f

//------------------------------------------------------------------------------
// XBEE Config
// Define USE_XBEE to cause all debug prints to go through the xbee
// Comment out the line to debug over usb
// #define USE_XBEE

// Xbee serial
#ifdef USE_XBEE
#define Serial Serial5
#endif

//------------------------------------------------------------------------------
// IMU Parameters
// Set enable flag to 1 to use accelerometer and gyro, set to 0 to just use gyro
#define IMU_ENABLE_COMPLEMENTARY_FILTER 0
#define IMU_COMPLEMENTARY_FILTER_TAU 0.95f

// Set above 0 to print imu debug messages
#define IMU_VERBOSE 0

// Pins for BNO080 IMU
#define SPI_CS_PIN 15
#define SPI_WAK_PIN 14
#define SPI_INTPIN 17
#define SPI_RSTPIN 16

//------------------------------------------------------------------------------
// Datalogger parameters
// Set to 1 to enable data logging, set to 0 to disable
#define ENABLE_DATALOGGER 0

// Set above 0 to print debugging messages
#define DATALOGGER_VERBOSE 0

#endif
