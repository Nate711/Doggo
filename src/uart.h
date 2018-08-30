#ifndef UART_H
#define UART_H

#include "ChRt.h"

extern THD_WORKING_AREA(waSerialThread, 128);
extern THD_FUNCTION(SerialThread, arg);

void ProcessPositionMsg(char* msg, int len);
void ProcessNLMessage(char* msg, size_t len);
enum RXState { IDLING, READ_LEN, READ_PAYLOAD, READ_PAYLOAD_UNTIL_NL};

#endif
