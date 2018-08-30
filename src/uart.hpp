#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "globals.h"
#include "config.h"


#ifndef UART_H
#define UART_H

//------------------------------------------------------------------------------
// SerialThread: receive serial messages from ODrive.
// Pulls bytes from the odrv0 serial buffer (aka Serial1) at a rate of 100khz.
// When a newline char is received, it calls parseEncoderMessage to update the
// struct associated with the serial buffer.

// TODO: add timeout behavior: throw out buffer if certain time has elapsed since
// a new message has started being received

void ProcessPositionMsg(char* msg, int len);
void ProcessNLMessage(char* msg, size_t len);
enum RXState { IDLING, READ_LEN, READ_PAYLOAD, READ_PAYLOAD_UNTIL_NL};
// 128 byte stack beyond task switch and interrupt needs.
static THD_WORKING_AREA(waSerialThread, 128);

static THD_FUNCTION(SerialThread, arg) {
    (void)arg;

    const int BUFFER_SIZE = 32;
    char msg[BUFFER_SIZE]; // running buffer of received characters
    size_t msg_idx = 0; // keep track of which index to write to
    RXState rx_state = IDLING;
    const uint8_t START_BYTE = 1;
    size_t payload_length = 0;

    odrv0Serial.clear();
    odrv0Serial.clear();

    long msg_start = 0;
    long msg_end = 0;
    int loop_iters = 0;

    while (true) {
        loop_iters++;
        while (odrv0Serial.available()) {
            // Read latest byte out of the serial buffer
            char c = odrv0Serial.read();
            switch (rx_state) {
                case IDLING:
                    if (c == START_BYTE) {
#ifdef DEBUG_LOW
                        msg_start = micros();
                        loop_iters = 0;
#endif
                        rx_state = READ_LEN;
                    }
                    break;
                case READ_LEN:
                    payload_length = c;
                    if (payload_length >= BUFFER_SIZE) {
#ifdef DEBUG_LOW
                        Serial << "Payload bigger than buffer!\n";
#endif
                        rx_state = IDLING;
                    } else if (payload_length == 0) {
                        rx_state = READ_PAYLOAD_UNTIL_NL;
                    } else {
                        rx_state = READ_PAYLOAD;
                    }
                    break;
                case READ_PAYLOAD_UNTIL_NL:
                    msg[msg_idx++] = c;

                    if (c == '\n') {
                        if (msg_idx < BUFFER_SIZE) {
                            msg[msg_idx] = '\0'; // null terminate to form string
                            ProcessNLMessage(msg,msg_idx);
                        }
                        rx_state = IDLING;
                        msg_idx = 0;
                        payload_length = 0;
#ifdef DEBUG_LOW
                        msg_end = micros();
                        Serial << "rcvd in: " << msg_end - msg_start << " in " << loop_iters << " loops\n";
                        // NOTE: As of code 7/7/18, the average receive time was 282us
                        // And number of loop executions to get a message was 34
#endif
                    }
                    break;
                case READ_PAYLOAD:
                    msg[msg_idx++] = c;

                    if (msg_idx == payload_length) {
                        if (msg[0] == 'P') {
                            ProcessPositionMsg(msg,msg_idx);
                        }
                        rx_state = IDLING;
                        msg_idx = 0;
                        payload_length = 0;
#ifdef DEBUG_LOW
                        msg_end = micros();
                        Serial << "rcvd in: " << msg_end - msg_start << " in " << loop_iters << " loops\n";
                        // NOTE: As of code 7/7/18, the average receive time was 282us
                        // And number of loop executions to get a message was 34
                        // NOTE: after finishing protocol (still crash bug), average
                        // receive time is ~440us in around 30 loops
#endif
                    }
                    break;
            }
        }
        // Run the serial checking loop at 100khz by delaying 10us
        // chThdSleepMilliseconds(10);
        // TODO: make this interrupt driven?
        // NOTE: using yield instead made the whole teensy crash, not sure why....
        chThdSleepMicroseconds(1000000/UART_FREQ);
    }
}

/**
 * Parse a dual position message and store the result in the odrive struct
 * @param msg char* : message
 * @param len int   : message length
 *
 * TODO: make it generalizable to other odrives and other odriveInterfaces
 */
void ProcessPositionMsg(char* msg, int len) {
    // only print the message if DEBUG is defined above
#ifdef DEBUG_LOW
    Serial.print("MSG RECEIVED: ");
    for(int i=0; i<len; i++) {
        Serial << (int)msg[i] << "(" << msg[i] <<") ";
    }
    Serial << "\n";
#endif
#ifdef DEBUG_HIGH
    Serial <<  "rcv at: " << micros() << '\n';
#endif

    float m0,m1;
    int result = odrv0Interface.ParseDualPosition(msg, len, m0, m1);
    // result: 1 means success, -1 means didn't get proper message
    if (result == 1) {
        // Update raw counts
        odrv0.axis0.pos_estimate = m0;
        odrv0.axis1.pos_estimate = m1;

#ifdef DEBUG_LOW
        Serial << "M0,M1 inc: " << m0 << " " << m1 << '\n';
#endif

        // TODO: this calculation of absolute pos is in the wrong function / scope
        // Update absolute positions
        odrv0.axis0.abs_pos_estimate = m0 + odrv0.axis0.ENCODER_OFFSET;
        odrv0.axis1.abs_pos_estimate = m1 + odrv0.axis1.ENCODER_OFFSET;

        // NOTE: it's possible that the feedback delay is wrong if an old
        // encoder reading took so long to come back that it came back after
        // another current command was sent; ie:
        // 1) Teensy sends command I1
        // 2) Teensy sends command I2
        // 3) Teensy receives reading E1
        // 4) Teensy thinks delay was E1-I2 when in reality it was E1 - I1
        // This problem won't happen if the delay is short and the control rate is small
        latest_receive_timestamp = micros();
        global_debug_values.feedback_loop_time = latest_receive_timestamp - latest_send_timestamp;
#ifdef DEBUG_HIGH
        Serial << "Done prs at: " << micros() << '\n';
        Serial << "Comm loop(uS): " << global_debug_values.feedback_loop_time << "\n";
        // NOTE: As of 7/7/18 code, around 1500us from send to receive
#endif
    } else {
        // TODO put a debug flag somewhere, otherwise printing messages like
        // these will probably screw things up
#ifdef DEBUG_LOW
        Serial.println("Parse failed. Wrong message length or bad checksum.");
#endif
    }
}
#endif

void ProcessNLMessage(char* msg, size_t len) {
    Serial << "Received NL message: " << msg;
}
