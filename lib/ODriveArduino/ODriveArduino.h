
#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"

class ODriveArduino {
public:
    enum AxisState_t {
        AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
    };

    ODriveArduino(HardwareSerial& serial);

    // Commands
    void SetDualCurrent(float current0, float current1);
    void SetCoupledPosition(float theta, float gamma);
    void SetCoupledPosition(float sp_theta, float sp_gamma, struct LegGain gains);
    void SetCoupledPosition(struct LegGain gains);
    void SetCurrent(int motor_number, float current);
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);

    void SetProperty(char* property, char* value);
    void ReadProperty(char* property);
    void QueryVBusVoltage();

    void SetCurrentLims(float current_lim);
    void ReadCurrents();

    // Protocol functions
    static int ParseDualPosition(char* msg, int len, float& m0, float& m1);

    // General params
    float readFloat();
    String readString();
    int32_t readInt();

    // State helper
    bool run_state(int axis, int requested_state, bool wait);
private:
    HardwareSerial& serial_;
    void SendNLLen();
    void SendStartByte();
    void SendByte(uint8_t byte);
    void SendShort(int16_t val);

    const char START_BYTE = 1;
    const char NL_LEN = 0;
};

// Struct to hold PID gains for the legs
struct LegGain {
    float kp_theta = 0;
    float kd_theta = 0;

    float kp_gamma = 0;
    float kd_gamma = 0;
};

#endif //ODriveArduino_h
