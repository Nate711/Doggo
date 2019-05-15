#include "usb_serial.h"
#include "ChRt.h"
#include "Arduino.h"
#include "globals.h"
#include "config.h"
#include "jump.h"
#include "backflip.h"
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
            if (c == ';' || c == '\n') {
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
    // Note: Putting a space in front of %c allows you type commands like:
    // f 2.0; l 0.01; h 0.08
    int num_parsed = sscanf(cmd, " %c %f", &c, &f);
    if (num_parsed < 1) {
        Serial.println("Invalid command");
        return;
    }
    switch(c) {
        // Change gait frequency
        case 'f':
            Serial << "Set freq. to: " << f << "\n";
            state_gait_params[state].freq = f;
            break;
        // Change stride length
        case 'l':
            Serial << "Set stride len to: " << f << "\n";
            state_gait_params[state].step_length = f;
            break;
        // Change stance height
        case 'h':
            Serial << "Set stance ht. to: " << f << "\n";
            state_gait_params[state].stance_height = f;
            break;
        // Change gait up amplitude
        case 'u':
            Serial << "Set up amp. to: " << f << "\n";
            state_gait_params[state].up_amp = f;
            break;
        // Change gait down amplitude
        case 'd':
            Serial << "Set down amp. to: " << f << "\n";
            state_gait_params[state].down_amp = f;
            break;
        // Change gait flight percent
        case 'p':
            Serial << "Set flt. perc. to: " << f << "\n";
            state_gait_params[state].flight_percent = f;
            break;
        // Change leg gains
        case 'g':
            { // Have to create a new scope here in order to declare variables
                float kp_t, kd_t, kp_g, kd_g;
                int res = sscanf(cmd, "g %f %f %f %f", &kp_t, &kd_t, &kp_g, &kd_g);
                if (res == 4) {
                    Serial << "Set gains to: " << kp_t << " " << kd_t << " " << kp_g << " " << kd_g << "\n";
                    gait_gains.kp_theta = kp_t;
                    gait_gains.kd_theta = kd_t;
                    gait_gains.kp_gamma = kp_g;
                    gait_gains.kd_gamma = kd_g;
                } else {
                    Serial.println("Invalid gain format.");
                }
            }
            break;
        // Toggle debug printing
        case 'D':
            enable_debug = !enable_debug;
            Serial << "Debug printing: " << enable_debug << "\n";
            break;
        // Switch into STOP state
        case 'S':
            state = STOP;
            Serial.println("STOP");
            break;
        // Switch into DANCE state
        case 'E':
            TransitionToDance();
            break;
        // Switch into BOUND state
        case 'B':
            TransitionToBound();
            break;
        // Switch into TROT state
        case 'T':
            TransitionToTrot();
            break;
        // Switch into WALK state
        case 'W':
            TransitionToWalk();
            break;
        // Switch into WALK state
        case 'P':
            TransitionToPronk();
            break;
        // Switch into JUMP state
        case 'J':
            StartJump(millis()/1000.0f);
            Serial.println("JUMP");
            break;
        case 'H':
            TransitionToHop();
            break;
        case 'F':
            StartFlip(millis()/1000.0f);
            break;
        case 'R':
            state = RESET;
            Serial.println("RESET");
            break;
        // // Switch into TEST state
        // TODO: Make new character for test mode
        case '1':
            state = TEST;
            Serial.println("(1)TEST");
            break;
        default:
            Serial.println("Unknown command");
    }
}

void PrintGaitCommands() {
    Serial.println("Available gait commands:");
    Serial.println("(f)req");
    Serial.println("step (l)ength");
    Serial.println("stance (h)eight");
    Serial.println("(d)own amplitude");
    Serial.println("(u)p amplitude");
    Serial.println("flight (p)roportion");
}

void PrintStates() {
    Serial.println("STATES: Danc(E), (W)alk, (T)rot, (B)ound, (P)ronk, (S)top, (J)ump");
    Serial.println("Toggle (D)ebug");
}
