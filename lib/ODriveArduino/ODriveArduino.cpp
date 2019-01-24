#include "Arduino.h"
#include "ODriveArduino.h"

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

/**
 * Construct ODriveArduino object linked to the given serial port.
 * @param serial Serial port to use to communicate to the ODrive
 */
ODriveArduino::ODriveArduino(HardwareSerial& serial)
: serial_(serial) {}

/**
 * Set an ODrive property from the Teensy
 * @param property The ODrive property to set
 * @param value    The value for the property
 *
 * Example: odrv0.SetProperty("axis0.motor.config.current_lim", "5.0f");
 */
void ODriveArduino::SetProperty(char* property, char* value) {
    SendStartByte(); SendNLLen();
    serial_ << "w " << property << " " << value << '\n';
}

/**
 * Read an ODrive property from the Teensy
 * @param property Property to query
 * NOTE: You must somehow handle the reponse from the ODrive separately
 */
void ODriveArduino::ReadProperty(char* property) {
    SendStartByte(); SendNLLen();
    serial_ << "r " << property << "\n";
}

/**
 * Set the current limits for both motors.
 * @param current_lim Current limit
 */
void ODriveArduino::SetCurrentLims(float current_lim) {
    SendStartByte(); SendNLLen();
    serial_ << "w axis0.motor.config.current_lim " << current_lim << "\n";
    SendStartByte(); SendNLLen();
    serial_ << "w axis1.motor.config.current_lim " << current_lim << "\n";
}

void ODriveArduino::ReadCurrents() {
    SendStartByte(); SendNLLen();
    serial_ << "r axis0.motor.current_control.Iq_measured\n";
    SendStartByte(); SendNLLen();
    serial_ << "r axis1.motor.current_control.Iq_measured\n";
}

/**
 * Send a message to the odrive that tells it to send back the vbus voltage
 * Working as of 7/7/18
 */
void ODriveArduino::QueryVBusVoltage() {
    SendStartByte(); SendNLLen();
    serial_ << "r vbus_voltage\n";
}
/**
* Parses the encoder position message and stores positions as counts
* Assumes the message is in format "<1><6><'P'><short1><short2><checksum>"

* TODO: add pll_vel to the message for better motor velocity measurement!!!
* This would greatly improve the noise on the Kd term!!
* @param msg    String: Message to parse
* @param th     float&: Output parameter for theta reading
* @param ga     float&: Output parameter for gamma reading
* @return       int:    1 if success, -1 if failed to find get full message or checksum failed
*/
int ODriveArduino::ParseDualPosition(char* msg, int len, float& th, float& ga) {
    // check if 1 byte for "P", 4 bytes holding encoder data, and 1 checksum byte were received
    if (len != 6) {
        return -1; // return -1 to indicate that the message length was wrong
    } else {
        // retrieve short from byte stream
        // remember that the first character is 'P'
        uint16_t th_16 = (msg[2] << 8) | msg[1];
        uint16_t ga_16 = (msg[4] << 8) | msg[3];
        uint8_t rcvdCheckSum = msg[5];

        // compute checksum
        uint8_t checkSum = 0;
        checkSum ^= msg[0]; // letter 'P'
        checkSum ^= msg[1];
        checkSum ^= msg[2];
        checkSum ^= msg[3];
        checkSum ^= msg[4];

        // DEBUG ONLY
        // Serial << "Comp checksum: " << (int)checkSum << " rcvd: " << (int)rcvdCheckSum << "\n";

        // if the computed and received check sums match then update the motor position variables
        if (checkSum == rcvdCheckSum) {
            // convert to float
            // TODO: figure out if casts are needed
            th = ((float) ((int16_t) th_16))/1000.0f;
            ga = ((float) ((int16_t) ga_16))/1000.0f;
            return 1;
        } else {
            // return -1 to indicate that the checksums didn't match
            return -1;
        }
    }
    return 1;
}

/**
 * Send the start byte to indicate to the receiver that a new message is being sent
 */
void ODriveArduino::SendStartByte() {
    serial_ << START_BYTE;
}

/**
 * Send the payload length value (0) that indicates to the receiver to read until a newline is hit
 */
void ODriveArduino::SendNLLen() {
    serial_ << NL_LEN;
}

/**
 * XOR operation on first 8 bits and last 8 bits of a short
 * @param  val  The short to work on
 * @return      Byte resulting from xoring the first 8 and last 8 bits.
 */
uint8_t XorShort(int16_t val) {
    // bad to duplicate code :(
    uint8_t v0 = val & 0xFF;
    uint8_t v1 = (val >> 8) & 0xFF;
    return v0 ^ v1;
}

/**
 * Send a single byte over serial
 * @param byte  Byte to send to the ODrive
 */
void ODriveArduino::SendByte(uint8_t byte) {
    serial_ << (char) byte;
}

/**
 * Send a short (16 bit signed int) over serial
 * @param val   The short to send to the ODrive
 */
void ODriveArduino::SendShort(int16_t val) {
    uint8_t v0 = val & 0xFF;
    uint8_t v1 = (val >> 8) & 0xFF;
    SendByte(v0);
    SendByte(v1);
}

/**
 * Sends a command for both motor currents in the form "<1><6>C<i0bytes><i1bytes><checksum>".
 * @param current0      Desired current for motor 0
 * @param current1      Desired current for motor 1
 */
void ODriveArduino::SetDualCurrent(float current0, float current1) {
    // NOTE: Desired current values will be send as their value times 100!
    // Ie, 5.6A => 560
    // The limit to the multiplier is dictated by 32000/maximum amps
    // So if we want 100A maximum value, then the mult could be 320
    const int MULTIPLIER = 100;

    // constrain the current set points so they don't exceed the limits of int16
    current0 = constrain(current0, -30000/MULTIPLIER, 30000/MULTIPLIER);
    current1 = constrain(current1, -30000/MULTIPLIER, 30000/MULTIPLIER);
    int16_t i0_16 = (current0 * MULTIPLIER);
    int16_t i1_16 = (current1 * MULTIPLIER);

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
 * Sends a command for a coupled position in the form "<1><6>P<theta_bytes><gamma_bytes><checksum>".
 * @param theta      Desired theta setpoint
 * @param gamma      Desired gamma setpoint
 */
void ODriveArduino::SetCoupledPosition(float theta, float gamma) {

    const int MULTIPLIER = 1000;

    int16_t theta_16 = (theta * MULTIPLIER);
    int16_t gamma_16 = (gamma * MULTIPLIER);

    // Calculate the checksum based on the 2 current value shorts
    uint8_t checkSum = 'P';
    checkSum ^= XorShort(theta_16);
    checkSum ^= XorShort(gamma_16);

    // Send off bytes
    SendStartByte(); // send start byte
    SendByte(6); // payload length
    SendByte('P'); // dual current command
    SendShort(theta_16);
    SendShort(gamma_16);
    SendByte(checkSum);
}

void ODriveArduino::SetCoupledPosition(float sp_theta, float sp_gamma, struct LegGain gains) {

    const int POS_MULTIPLIER = 1000;
    const int GAIN_MULTIPLIER = 100;

    int16_t sp_theta_16 = (sp_theta * POS_MULTIPLIER);
    int16_t kp_theta_16 = (gains.kp_theta * GAIN_MULTIPLIER);
    int16_t kd_theta_16 = (gains.kd_theta * GAIN_MULTIPLIER);

    int16_t sp_gamma_16 = (sp_gamma * POS_MULTIPLIER);
    int16_t kp_gamma_16 = (gains.kp_gamma * GAIN_MULTIPLIER);
    int16_t kd_gamma_16 = (gains.kd_gamma * GAIN_MULTIPLIER);

    // Calculate the checksum based on the 2 current value shorts
    uint8_t checkSum = 'S';
    checkSum ^= XorShort(sp_theta_16);
    checkSum ^= XorShort(kp_theta_16);
    checkSum ^= XorShort(kd_theta_16);
    checkSum ^= XorShort(sp_gamma_16);
    checkSum ^= XorShort(kp_gamma_16);
    checkSum ^= XorShort(kd_gamma_16);

    // Send off bytes
    SendStartByte(); // send start byte
    SendByte(14); // payload length
    SendByte('S'); // dual current command
    SendShort(sp_theta_16);
    SendShort(kp_theta_16);
    SendShort(kd_theta_16);
    SendShort(sp_gamma_16);
    SendShort(kp_gamma_16);
    SendShort(kd_gamma_16);
    SendByte(checkSum);
}

void ODriveArduino::SetCoupledPosition(struct LegGain gains) {
    // TODO: correct but where S command doesn't get enough parameters
}

/**
 * Send a current command for single axis
 * @param motor_number  Axis number (0 or 1)
 * @param current       Float amount of current
 */
void ODriveArduino::SetCurrent(int motor_number, float current) {
    SendStartByte(); SendNLLen();
    serial_ << "c " << motor_number << " " << current << "\n";
}

/**
 * Sends a position command to the ODrive. Once the ODrive gets this command
 * then it will use its own PID control to get the motor to that position
 * @param motor_number  Axis number (0 or 1), Indicate which motor to command
 * @param position      Desired position in counts
 */
void ODriveArduino::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

/**
 * Sends a position command to the ODrive and set a velocity feedforward gain.
 * @param motor_number          Axis number (0 or 1), Indicate which motor to command
 * @param position              Desired position in counts
 * @param velocity_feedforward  Feed forward term
 */
void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

/**
 * Sends a position command along with velocity feedforward and current feedforward
 * @param motor_number         Axis number (0 or 1), Indicate which motor to command
 * @param position             Desired position in counts
 * @param velocity_feedforward Velocity feed foward
 * @param current_feedforward  Current feed forward
 */

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    SendStartByte(); SendNLLen();
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

/**
 * Sends a velocity command to the ODrive
 * @param motor_number Axis number (0 or 1)
 * @param velocity     Desired velocity
 */
void ODriveArduino::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

/**
 * Sends a velocity command to the ODrive with current feedforward
 * @param motor_number        Axis number (0 or 1)
 * @param velocity            Desired velocity
 * @param current_feedforward Current feedfoward
 */
void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    SendStartByte(); SendNLLen();
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

/**
 * VERY DUBIOUS / COMPLETELY UNTESTED
 */

/**
 * Attempt to read a float sent from the ODrive
 * @return Parsed float. TODO: handle error
 */
float ODriveArduino::readFloat() {
    return readString().toFloat();
}

/**
 * Attempt to read an integer sent from the ODrive
 * @return Parsed integer. TODO: handle error
 */
int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

/**
 * Command a certain desired state for a given axis
 * @param  axis            Axis number (0 or 1)
 * @param  requested_state Desired state, see state ENUM for viable options
 * @param  wait            Block program until ODrive is back to idle?
 * @return                 Did ODrive successfully reach desired state?
 */
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

/**
 * Reads a response from the ODrive. Only works if response ends in a '\n'!!
 * NOTE: Make sure that the serial thread isn't running concurrently as this function
 * or else both codes will try to read from the serial port at the same time!
 * @return Received string
 */
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
        if (c == 0 || c == 1) { // ignore character if its the start byte or length byte
            continue;
        }
        str += c;
    }
    return str;
}
