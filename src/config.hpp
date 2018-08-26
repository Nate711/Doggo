#ifndef CONFIG_H
#define CONFIG_H

// Define DEBUG_LOW and/or DEBUG_HIGH to enable printing debug messages
// I've made two flags so you have more options in terms of which messages to show
#define DEBUG_HIGH
#define DEBUG_LOW
// #define PRINT_ONCE

// Important: Thread execution frequencies
const int POSITION_CONTROL_FREQ = 100;
const int DEBUG_PRINT_FREQ = 100;
const int UART_FREQ = 20000;
const int USB_SERIAL_FREQ = 10;
const int DATALOG_FREQ = 100;

#endif
