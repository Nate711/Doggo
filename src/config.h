#ifndef CONFIG_H
#define CONFIG_H

// Define USE_XBEE to cause all debug prints to go through the xbee
// Comment out the line to debug over usb
// #define USE_XBEE


// Define DEBUG_LOW and/or DEBUG_HIGH to enable printing debug messages
// I've made two flags so you have more options in terms of which messages to show
//#define DEBUG_HIGH
//#define DEBUG_LOW
// #define PRINT_ONCE

// Important: Thread execution frequencies
#define POSITION_CONTROL_FREQ 400
#define DEBUG_PRINT_FREQ 20
#define UART_FREQ 2000
#define USB_SERIAL_FREQ 100

#define CURRENT_LIM 40.0f


//------------------------------------------------------------------------------
// Xbee serial
#ifdef USE_XBEE
#define Serial Serial5
#endif

#endif
