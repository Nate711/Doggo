#include "usb_serial.h"
#include "ChRt.h"
#include "Arduino.h"
#include "globals.h"
#include "config.h"
#include "jump.h"
#include "position_control.h"

THD_WORKING_AREA(waUSBSerialThread, 2048);

THD_FUNCTION(USBSerialThread, arg) {
    (void)arg;

    int MAX_COMMAND_LENGTH = 32;
    char cmd[MAX_COMMAND_LENGTH + 1];
    int pos = 0;

    while(true) {
        while(Serial.available()) {
            char c = Serial.read();
            if (c == ' ' || c == '\n') {
                cmd[pos] = '\0';
                InterpretCommand(cmd);
                pos = 0;
            } else {
                cmd[pos++] = c;
            }
        }

        chThdSleepMicroseconds(1000000/USB_SERIAL_FREQ);
    }
}

void InterpretCommand(char* cmd) {
    char c;
    float f;
    sscanf(cmd, "%c%f", &c, &f);
    switch(c) {
        case 'F':
            gaitParams.FREQ = f;
            break;
        case 'l':
            gaitParams.step_length = f;
            break;
        case 'h':
            gaitParams.stance_height = f;
            break;
        case 'u':
            gaitParams.up_AMP = f;
            break;
        case 'd':
            gaitParams.down_AMP = f;
            break;
        case 'p':
            gaitParams.flight_percent = f;
        case 'g':
            sscanf(cmd, "g%f,%f,%f,%f", &(gaitGains.kp_theta), &(gaitGains.kd_theta), &(gaitGains.kp_gamma), &(gaitGains.kd_gamma));
            break;
        case 'j':
            StartJump(millis()/1000.0f);
            Serial.println("Jump");
            break;
        default:
            Serial.println("Unknown command");
    }
}
