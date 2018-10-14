#include "uart.h"
#include "ChRt.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "globals.h"
#include "config.h"

//------------------------------------------------------------------------------
// SerialThread: receive serial messages from ODrive.
// Pulls bytes from the odrv0 serial buffer (aka Serial1) at a rate of 100khz.
// When a newline char is received, it calls parseEncoderMessage to update the
// struct associated with the serial buffer.

// TODO: add timeout behavior: throw out buffer if certain time has elapsed since
// a new message has started being received

THD_WORKING_AREA(waSerialThread, 2048);

THD_FUNCTION(SerialThread, arg) {
    (void)arg;

    const int BUFFER_SIZE = 32;
    char msg[BUFFER_SIZE]; // running buffer of received characters
    size_t msg_idx = 0; // keep track of which index to write to
    RXState rx_state = IDLING;
    const uint8_t START_BYTE = 1;
    size_t payload_length = 0;

    odrv0Serial.clear();
    odrv1Serial.clear();

    long msg_start = 0;
    long msg_end = 0;
    int loop_iters = 0;

    while (true) {
        loop_iters++;
        while (odrv1Serial.available()) {
            // Read latest byte out of the serial buffer
            char c = odrv1Serial.read();
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
                        // NOTE: As of code 10/14/18, the average receive time was 1100
                        // And number of loop executions to get a message was 0 or 1
#endif



                    }
                    break;
            }
        }

        // TODO: make this interrupt driven?
        // NOTE: using yield instead made the whole teensy crash, not sure why....
        chThdSleepMicroseconds(1000000/UART_FREQ);
    }
}

/**
 * Parse a theta/gamma message from odrv0 and store the result in the odrive struct
 * @param msg char* : message
 * @param len int   : message length
 *
 * TODO: make it generalizable to other odrives and other odriveInterfaces
 */
void ProcessPositionMsg(char* msg, int len) {
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

    float th,ga;
    int result = ODriveArduino::ParseDualPosition(msg, len, th, ga);
    // result: 1 means success, -1 means didn't get proper message
    if (result == 1) {
        // Update theta and gamma
        global_debug_values.odrv0.est_theta = th;
        global_debug_values.odrv0.est_gamma = ga;

#ifdef DEBUG_LOW
        Serial << "Th,Ga: " << th << " " << ga << '\n';
#endif

        // NOTE: it's possible that the feedback delay is wrong if an old
        // encoder reading took so long to come back that it came back after
        // another current command was sent; ie:
        // 1) Teensy sends command I1
        // 2) Teensy sends command I2
        // 3) Teensy receives reading E1
        // 4) Teensy thinks delay was E1-I2 when in reality it was E1 - I1
        // This problem won't happen if the delay is short and the control rate is small
        //
        latest_receive_timestamp = micros();
        global_debug_values.position_reply_time = latest_receive_timestamp - latest_send_timestamp;

#ifdef DEBUG_HIGH
        Serial << "Done prs at: " << micros() << '\n';
        Serial << "Reply time (uS): " << global_debug_values.position_reply_time << "\n";
        // NOTE: As of 7/7/18 code, around 1500us from send to receive
#endif

    } else {

#ifdef DEBUG_LOW
        Serial.println("Parse failed. Wrong message length or bad checksum.");
#endif

    }
}

void ProcessNLMessage(char* msg, size_t len) {
    Serial << msg;
    // Serial << "Received NL message: " << msg;
}
