#include "backflip.h"
#include "ODriveArduino.h"
#include "globals.h"
#include "position_control.h"

void pointDown() {
    float pitch = global_debug_values.imu.pitch;
    if (pitch > M_PI/2 || pitch < -M_PI/2) return;
    float y = gait_params.stance_height;
    float theta, gamma;
    CartesianToThetaGamma(0.0, y, 1, theta, gamma);
    odrv0Interface.SetCoupledPosition(pitch, gamma, gait_gains);
    odrv1Interface.SetCoupledPosition(pitch, gamma, gait_gains);
    odrv2Interface.SetCoupledPosition(-pitch, gamma, gait_gains);
    odrv3Interface.SetCoupledPosition(-pitch, gamma, gait_gains);
}
