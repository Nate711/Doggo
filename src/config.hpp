#ifndef CONFIG_H
#define CONFIG_H

// define debug to enable printing debug messages
// I've made two flags so you have more options in terms of which messages to show
// #define DEBUG_HIGH
// #define DEBUG_LOW

// thread execution frequencies
const int POSITION_CONTROL_FREQ = 100;
const int DEBUG_PRINT_FREQ = 100;
const int UART_FREQ = 10000;

#endif
