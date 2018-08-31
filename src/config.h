#ifndef CONFIG_H
#define CONFIG_H

// Define DEBUG_LOW and/or DEBUG_HIGH to enable printing debug messages
// I've made two flags so you have more options in terms of which messages to show
#define DEBUG_HIGH
#define DEBUG_LOW
// #define PRINT_ONCE

// Important: Thread execution frequencies
#define POSITION_CONTROL_FREQ 100
#define DEBUG_PRINT_FREQ 100
#define UART_FREQ 20000
#define USB_SERIAL_FREQ 10
#define DATALOG_FREQ 100

#endif
