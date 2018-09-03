#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include "ChRt.h"

extern THD_WORKING_AREA(waUSBSerialThread, 128);
extern THD_FUNCTION(USBSerialThread, arg);

void InterpretCommand(char* cmd);

#endif
