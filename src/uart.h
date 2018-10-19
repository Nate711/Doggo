#ifndef UART_H
#define UART_H

#include "ChRt.h"
#include "Arduino.h"
#include "globals.h"

extern THD_WORKING_AREA(waSerialThread, 2048);
extern THD_FUNCTION(SerialThread, arg);

void ProcessPositionMsg(char* msg, int len, HardwareSerial& odrvSerial, struct MsgOutput& odrvMsgOutput);
void ProcessNLMessage(char* msg, size_t len);
enum RXState { IDLING, READ_LEN, READ_PAYLOAD, READ_PAYLOAD_UNTIL_NL};
void ProcessSerial(HardwareSerial& odrvSerial, struct MsgParams& odrvMsgParams, struct MsgOutput& odrvMsgOutput);

const int BUFFER_SIZE_ = 32;
struct MsgParams {
	const int BUFFER_SIZE = BUFFER_SIZE_;
	const uint8_t START_BYTE = 1;
  char msg[BUFFER_SIZE_]; // running buffer of received characters
  size_t msg_idx = 0; // keep track of which index to write to
  RXState rx_state = IDLING;
  size_t payload_length = 0;
  long msg_start = 0;
  long msg_end = 0;
  int loop_iters = 0;
};

struct MsgOutput{
	float* theta;
	float* gamma;
};


#endif
