#include "usb_serial.h"
#include "ChRt.h"
#include "Arduino.h"
#include "globals.h"
#include "config.h"
#include "jump.h"

THD_WORKING_AREA(waUSBSerialThread, 128);

THD_FUNCTION(USBSerialThread, arg) {
    (void)arg;

    int MAX_COMMAND_LENGTH = 256;
    char cmd[MAX_COMMAND_LENGTH];
    int pos = 0;

    while(true) {
        while(Serial.available()) {
            char c = Serial.read();
            cmd[pos++] = c;
            if (c == ' ' || c == '\n') {
                InterpretCommand(cmd);
                pos = 0;
            }
        }

        chThdSleepMicroseconds(1000000/USB_SERIAL_FREQ);
    }
}

void InterpretCommand(char* cmd) {
    char c;
    float f;
    sscanf(cmd, "%c%f ", &c, &f);

    switch(c) {
        case 'j':
            StartJump(millis()/1000.0f);
            Serial.println("Jump");
            break;
        default:
            Serial.println("Unknown command");
    }
}
