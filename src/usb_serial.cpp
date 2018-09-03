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
                Serial << "cmd " << cmd << "\n";
                InterpretCommand(cmd);
                pos = 0;
                Serial.println("Success");
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
    Serial << c << " " << f << "\n";
    switch(c) {
        case 'F':
            gaitParams.FREQ = f;
            break;
        case 'l':
            gaitParams.stepLength = f;
            break;
        case 'h':
            gaitParams.stanceHeight = f;
            break;
        case 'u':
            gaitParams.upAMP = f;
            break;
        case 'd':
            gaitParams.downAMP = f;
            break;
        case 'p':
            gaitParams.flightPercent = f;
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
