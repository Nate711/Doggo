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
        case 'f':
            gait_params.FREQ = f;
            break;
        case 'l':
            gait_params.step_length = f;
            break;
        case 'h':
            gait_params.stance_height = f;
            break;
        case 'u':
            gait_params.up_AMP = f;
            break;
        case 'd':
            gait_params.down_AMP = f;
            break;
        case 'p':
            gait_params.flight_percent = f;
        case 'g':
            { // Have to create a new scope here in order to declare variables
                float kp_t, kd_t, kp_g, kd_g;
                int res = sscanf(cmd, "g%f,%f,%f,%f", &kp_t, &kd_t, &kp_g, &kd_g);
                if (res == 4) {
                    gait_gains.kp_theta = kp_t;
                    gait_gains.kd_theta = kd_t;
                    gait_gains.kp_gamma = kp_g;
                    gait_gains.kd_gamma = kd_g;
                }
            }
            break;
        case 'S':
            state = STOP;
            break;
        case 'G':
            state = GAIT;
            break;
        case 'J':
            StartJump(millis()/1000.0f);
            Serial.println("Jump");
            break;
        case 'T':
            state = TEST;
        default:
            Serial.println("Unknown command");
    }
}
