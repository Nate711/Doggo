#include "backflip.h"
#include "ODriveArduino.h"
#include "globals.h"
#include "position_control.h"
#include "imu.h"

float flip_start_time_ = 0.0f;

enum LegSet {
    FRONT,
    BACK
};

void pointDown(struct GaitParams params) {
    float pitch = global_debug_values.imu.pitch;
    if (pitch > M_PI/2 || pitch < -M_PI/2) return;
    float y = params.stance_height;
    float theta, gamma;
    CartesianToThetaGamma(0.0, y, 1, theta, gamma);
    odrv0Interface.SetCoupledPosition(pitch, gamma, gait_gains);
    odrv1Interface.SetCoupledPosition(pitch, gamma, gait_gains);
    odrv2Interface.SetCoupledPosition(-pitch, gamma, gait_gains);
    odrv3Interface.SetCoupledPosition(-pitch, gamma, gait_gains);
}

void StartFlip(float start_time_s) {
    state = FLIP;
    IMUTarePitch();
    Serial.println("FLIP");
    flip_start_time_ = start_time_s;
    UpdateStateGaitParams(FLIP);
    gait_gains = {120,1,140,1};
    PrintGaitParams();
}

void CommandLegsThetaY(float theta, float r, struct LegGain lg, LegSet ls) {
    float t_,gamma; // theta, gamma
    CartesianToThetaGamma(0.0, r, 1, t_, gamma);

    if (ls == FRONT) {
        odrv0Interface.SetCoupledPosition(theta, gamma, lg);
        odrv3Interface.SetCoupledPosition(-theta, gamma, lg);

        global_debug_values.odrv0.sp_theta = theta;
        global_debug_values.odrv0.sp_gamma = gamma;
        global_debug_values.odrv3.sp_theta = theta;
        global_debug_values.odrv3.sp_gamma = gamma;
    }
    else if (ls == BACK){
        odrv1Interface.SetCoupledPosition(theta, gamma, lg);
        odrv2Interface.SetCoupledPosition(-theta, gamma, lg);

        global_debug_values.odrv1.sp_theta = theta;
        global_debug_values.odrv1.sp_gamma = gamma;
        global_debug_values.odrv2.sp_theta = theta;
        global_debug_values.odrv2.sp_gamma = gamma;
    }
}

void ExecuteFlip(struct GaitParams params) {
    const float prep_time = 0.6f; // Duration before jumping [s]
    const float launch_time = 0.1f; // Duration before retracting the leg [s]

    struct LegGain rear_gains = {120,1,80,1};
    float rear_up_len = 0.08;
    float rear_down_len = 0.24;

    float t = millis()/1000.0f - flip_start_time_;
    float pitch = global_debug_values.imu.pitch;

    float theta=0,gamma=0;

    // Preparing to jump
    if (t < prep_time) {
        float y_back = rear_up_len;
        CommandLegsThetaY(pitch, y_back, rear_gains, BACK);

        float y_front = params.stance_height - params.up_amp;
        CommandLegsThetaY(pitch, y_front, gait_gains, FRONT);

    // Push off with front feet
    } else if (t >= prep_time && t < prep_time + launch_time) {
        float y_back = rear_up_len;
        CommandLegsThetaY(pitch, y_back, rear_gains, BACK);

        float y_front = params.stance_height + params.down_amp;
        CommandLegsThetaY(pitch, y_front, gait_gains, FRONT);

    // Rotate front legs to catch
} else if (pitch < 90.0*M_PI/180.0) {
        float y_back = rear_up_len;
        CommandLegsThetaY(pitch, y_back, rear_gains, BACK);

        float y_front = params.stance_height;
        CommandLegsThetaY(-pitch, y_front, gait_gains, FRONT);

    // Push off with back feet, keep rotating front legs to catch
    } else  if (pitch < 130.0*M_PI/180.0) {
        float y_back = rear_down_len;
        CommandLegsThetaY(pitch, y_back, gait_gains, BACK);

        float y_front = params.stance_height;
        CommandLegsThetaY(-pitch, y_front, gait_gains, FRONT);
    } else if (pitch < 180.0*M_PI/180.0){
        float y_back = params.stance_height;
        CommandLegsThetaY(pitch, y_back, gait_gains, BACK);

        float y_front = params.stance_height;
        CommandLegsThetaY(-pitch, y_front, gait_gains, FRONT);
    } else {
        float y_back = params.stance_height;
        CommandLegsThetaY(pitch, y_back, gait_gains, BACK);

        float y_front = params.stance_height;
        CommandLegsThetaY(pitch - 2*M_PI, y_front, gait_gains, FRONT);
    }
}
