
#include "Arduino.h"
#include "ODriveArduino.h"

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

#define DEBUG

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveArduino::ODriveArduino(HardwareSerial& serial)
: serial_(serial) {}

/**
* Parses the encoder position message and stores positions as counts
* Assumes the message is in format "<short1><short2><checksum>\n"

* TODO: add pll_vel to the message for better motor velocity measurement!!!
* This would greatly improve the noise on the Kd term!!
* @param msg   String: Message to parse
* @param m0    float&: Output parameter for axis0 reading
* @param m1    float&: Output parameter for axis1 reading
* @return      int:    1 if success, -1 if failed to find get full message or checksum failed
*/
int ODriveArduino::ParseDualPosition(char* msg, int len, float& m0, float& m1) {
    // check if 1 byte for "P", 4 bytes holding encoder data, and 1 checksum byte were received
    if (len != 6) {
        return -1; // error in message length
    } else {
        // retrieve short from byte stream
        // remember that the first character is 'P'
        uint16_t m0_16 = (msg[2] << 8) | msg[1];
        uint16_t m1_16 = (msg[4] << 8) | msg[3];
        uint8_t rcvdCheckSum = msg[5];

        // compute checksum
        uint8_t checkSum = 0;
        checkSum ^= msg[0]; // letter "P"
        checkSum ^= msg[1];
        checkSum ^= msg[2];
        checkSum ^= msg[3];
        checkSum ^= msg[4];

        // DEBUG ONLY
        #ifdef DEBUG
        Serial << "Comp checksum: " << (int)checkSum << " rcvd: " << (int)rcvdCheckSum << "\n";
        #endif

        // check if the check sum matched
        if (checkSum == rcvdCheckSum) {
            // convert to float
            // TODO: figure out if casts are needed
            m0 = (float) ((int16_t) m0_16);
            m1 = (float) ((int16_t) m1_16);

            #ifdef DEBUG
            Serial << "Parsed: " << m0 << "\t" << m1 << "\n";
            #endif
            return 1;
        } else {
            return -1;
        }
    }
    return 1;
}

/**
 * Send the start byte to indicate to the receiver that a new message is being sent
 */
void ODriveArduino::SendStartByte() {
    serial_ << (char)1;
}

/**
 * Send the payload length value (0) that indicates to the receiver to read until a newline is hit
 */
void ODriveArduino::SendNLLen() {
    serial_ << (char)0;
}

/**
 * xor operation on first 8 bits and last 8 bits of a short
 * @param  val : short
 * @return     : byte resulting from xor operation
 */
uint8_t XorShort(int16_t val) {
    // bad to duplicate code :(
    uint8_t v0 = val & 0xFF;
    uint8_t v1 = (val >> 8) & 0xFF;
    return v0 ^ v1;
}

/**
 * Send a single byte over serial
 * @param byte : Byte to send
 */
void ODriveArduino::SendByte(uint8_t byte) {
    serial_ << (char) byte;
}

/**
 * Send a short (16 bit signed int) over serial
 * @param val : Short to send
 */
void ODriveArduino::SendShort(int16_t val) {
    uint8_t v0 = val & 0xFF;
    uint8_t v1 = (val >> 8) & 0xFF;
    SendByte(v0);
    SendByte(v1);
    Serial << "i: " << v0 << v1<<'\n';
}

/**
 * Sends a command in the form "<1><6>C<i0bytes><i1bytes><checksum>".
 * @param current0 desired current for motor 0
 * @param current1 desired current for motor 1
 */
void ODriveArduino::SetDualCurrent(float current0, float current1) {
    // Serial  << "Sent: C " << current0 << " " << current1 << "\n";
    // serial_ << "C " << current0 << " " << current1 << "\n";

    // NOTE: Desired current values will be send as their value times 100!
    // Ie, 5.6A => 560
    // The limit to the multiplier is dictated by 32000/maximum amps
    // So if we want 100A maximum value, then the mult could be 320
    const int MULTIPLIER = 100;
    // constrain the current set points so they dont exceed the limits of int16
    current0 = constrain(current0, -30000/MULTIPLIER, 30000/MULTIPLIER);
    current1 = constrain(current1, -30000/MULTIPLIER, 30000/MULTIPLIER);
    int16_t i0_16 = (int16_t) (current0 * MULTIPLIER);
    int16_t i1_16 = (int16_t) (current1 * MULTIPLIER);

    // Calculate the checksum based on the 2 current value shorts
    uint8_t checkSum = 'C';
    checkSum ^= XorShort(i0_16);
    checkSum ^= XorShort(i1_16);

    // Send off bytes
    SendStartByte(); // send start byte
    SendByte(6); // payload length
    SendByte('C'); // dual current command
    SendShort(i0_16);
    SendShort(i1_16);
    SendByte(checkSum);
}

/**
 * Set current for single axis
 * @param motor_number : axis number (0 or 1)
 * @param current      : float amount of current
 */
void ODriveArduino::SetCurrent(int motor_number, float current) {
    SendStartByte(); SendNLLen();
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    SendStartByte(); SendNLLen();
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    SendStartByte(); SendNLLen();
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

float ODriveArduino::readFloat() {
    return readString().toFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait) {
    int timeout_ctr = 100;
    SendStartByte(); SendNLLen();
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';

    if (wait) {
        do {
            delay(100);

            SendStartByte(); SendNLLen();
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODriveArduino::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n') {
            break;
        }
        str += c;
    }
    return str;
}
