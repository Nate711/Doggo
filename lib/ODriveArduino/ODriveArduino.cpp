
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
    // check if the 4 bytes holding encoder data, 1 checksum byte, and 1 newline char were received
    if (len != 6) {
        return -1; // error in message length
    } else {
        // retrieve short from byte stream
        uint16_t m0_16 = (msg[1] << 8) | msg[0];
        uint16_t m1_16 = (msg[3] << 8) | msg[2];
        uint8_t rcvdCheckSum = msg[4];

        // compute checksum
        uint8_t checkSum = 0;
        checkSum ^= msg[0];
        checkSum ^= msg[1];
        checkSum ^= msg[2];
        checkSum ^= msg[3];

        // DEBUG ONLY
#ifdef DEBUG
        Serial << "Comp checksum: " << (int)checkSum << " rcvd: " << (int)rcvdCheckSum << "\n";
#endif

        // check if the check sum matched
        if (checkSum == rcvdCheckSum) {
            // convert to float
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

void ODriveArduino::SetCurrent(int motor_number, float current) {
  serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::SetDualCurrent(float current0, float current1) {
  Serial  << "Sent: C " << current0 << " " << current1 << "\n";
  serial_ << "C " << current0 << " " << current1 << "\n";
}

void ODriveArduino::SetPosition(int motor_number, float position) {
  SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
  SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
  serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity) {
  SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
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
  serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
  if (wait) {
    do {
      delay(100);
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
