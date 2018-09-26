#include "position_control.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "config.h"
#include "globals.h"
#include "jump.h"

//------------------------------------------------------------------------------
// PositionControlThread: Motor position control thread
// Periodically calculates result from PID controller and sends off new motor
// current commands to the ODrive(s)

// TODO: add support for multiple ODrives

THD_WORKING_AREA(waPositionControlThread, 512);

THD_FUNCTION(PositionControlThread, arg) {
    (void)arg;

    SetODriveCurrentLimits(CURRENT_LIM);
    chThdSleepMilliseconds(100);
    SetODriveCurrentLimits(CURRENT_LIM);

    while(true) {

        switch(state) {
            case STOP:
                {
                    // struct LegGain stop_gain = {80, 0.5, 80, 0.5};
                    float y1 = gait_params.stance_height;
                    float y2 = gait_params.stance_height;// + gait_params.down_AMP;
                    float theta1, gamma1, theta2, gamma2;
                    CartesianToThetaGamma(0.0, y1, 1, theta1, gamma1);
                    CartesianToThetaGamma(0.0, y2, 1, theta2, gamma2);
                    odrv0Interface.SetCoupledPosition(theta2, gamma2, gait_gains);
                    odrv1Interface.SetCoupledPosition(theta1, gamma1, gait_gains);
                    odrv2Interface.SetCoupledPosition(theta1, gamma1, gait_gains);
                    odrv3Interface.SetCoupledPosition(theta2, gamma2, gait_gains);
                }
                break;
            case DANCE:
                gait(gait_params, 0.0, 0.5, 0.0, 0.5, gait_gains);
                break;
            case BOUND:
                gait(gait_params, 0.0, 0.5, 0.5, 0.0, gait_gains);
                break;
            case TROT:
                gait(gait_params, 0.0, 0.5, 0.0, 0.5, gait_gains);
                break;
            case WALK:
                gait(gait_params, 0.0, 0.25, 0.75, 0.5, gait_gains);
                break;
            case PRONK:
                gait(gait_params, 0.0, 0.0, 0.0, 0.0, gait_gains);
                break;
            case JUMP:
                ExecuteJump();
                break;
            case ROTATE:
                {
                float theta,gamma;
                CartesianToThetaGamma(0, 0.24, 1.0, theta, gamma);
                float freq = 0.1;
                float phase = freq * (millis() - rotate_start)/1000.0f;
                theta = (-cos(2*PI * phase) + 1.0f) * 0.5 * 2 * PI;
                CommandAllLegs(theta, gamma, gait_gains);
                }
            case HOP:
                hop(gait_params);
                break;
            case TEST:
                test();
                break;
        }

        chThdSleepMicroseconds(1000000/POSITION_CONTROL_FREQ);
    }
}
long rotate_start = 0; // milliseconds when rotate was commanded
States state = STOP;

// {stance_height, down_AMP, up_AMP, flight_percent (proportion), step_length, FREQ}
struct GaitParams gait_params = {0.17, 0.04, 0.06, 0.35, 0.0, 2.5};
struct LegGain gait_gains = {80, 0.5, 50, 0.5};

/**
 * Set the current limits on both motors for all the odrives
 * @param limit Current limit
 * NOTE: sometimes a motor doesn't actually receive the command
 */
void SetODriveCurrentLimits(float limit) {
    odrv0Interface.SetCurrentLims(limit);
    odrv1Interface.SetCurrentLims(limit);
    odrv2Interface.SetCurrentLims(limit);
    odrv3Interface.SetCurrentLims(limit);
}


/**
* Here the Hip angles are assumed to be in radians with a 0 position in the
* downward y-direction and positive angles CCW.
*/

/**
* Takes the leg parameters and returns the gamma angle (rad) of the legs
*/
void GetGamma(float L, float theta, float& gamma) {
    float L1 = 0.09; // upper leg length (m)
    float L2 = 0.162; // lower leg length (m)
    float cos_param = (pow(L1,2.0) + pow(L,2.0) - pow(L2,2.0)) / (2.0*L1*L);
    if (cos_param < -1.0) {
        gamma = PI;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too small to find valid alpha and beta!");
        #endif
      } else if (cos_param > 1.0) {
        gamma = 0;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too large to find valid alpha and beta!");
        #endif
      } else {
        gamma = acos(cos_param);
      }
}

/**
* Converts the leg params L, gamma to cartesian coordinates x, y (in m)
* Set x_direction to 1.0 or -1.0 to change which direction the leg walks
*/
void LegParamsToCartesian(float L, float theta, float leg_direction, float& x, float& y) {
    x = leg_direction * L * cos(theta);
    y = L * sin(theta);
}

/**
* Converts the cartesian coords x, y (m) to leg params L (m), theta (rad)
*/
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta) {
    L = pow((pow(x,2.0) + pow(y,2.0)), 0.5);
    theta = atan2(leg_direction * x, y);
}

/**
* Sinusoidal trajectory generator function with flexibility from parameters described below. Can do 4-beat, 2-beat, trotting, etc with this.
*/
void SinTrajectory (float t, struct GaitParams params, float gaitOffset, float& x, float& y) {
    static float p = 0;
    static float prev_t = 0;

    float stanceHeight = params.stance_height;
    float downAMP = params.down_amp;
    float upAMP = params.up_amp;
    float flightPercent = params.flight_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    p += FREQ * (t - prev_t);
    prev_t = t;

    float gp = fmod((p+gaitOffset),1.0); // mod(a,m) returns remainder division of a by m
    if (gp <= flightPercent) {
        x = (gp/flightPercent)*stepLength - stepLength/2.0;
        y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;
    }
    else {
        float percentBack = (gp-flightPercent)/(1.0-flightPercent);
        x = -percentBack*stepLength + stepLength/2.0;
        y = downAMP*sin(PI*percentBack) + stanceHeight;
    }
}

void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma) {
    float L = 0.0;
    CartesianToLegParams(x, y, leg_direction, L, theta);
    GetGamma(L, theta, gamma);
    //Serial << "Th, Gam: " << theta << " " << gamma << '\n';
}

/**
 * Performs sanity checks on the odrive coupled gains to prevent obvious
 * and dangerous instabilities.
 * @param  gains LegGain to check
 * @return       True if valid gains, false if invalid
 */
bool IsValidLegGain(struct LegGain gains) {
    // check for unstable gains
    bool bad =  gains.kp_theta < 0 || gains.kd_theta < 0 ||
                gains.kp_gamma < 0 || gains.kd_gamma < 0;
    if (bad) {
        Serial.println("Invalid gains: <0");
        return false;
    }
    // check for instability / sensor noise amplification
    bad = bad || gains.kp_theta > 320 || gains.kd_theta > 10 ||
                 gains.kp_gamma > 320 || gains.kd_gamma > 10;
    if (bad) {
        Serial.println("Invalid gains: too high.");
        return false;
    }
    // check for underdamping -> instability
    bad = bad || (gains.kp_theta > 200 && gains.kd_theta < 0.1);
    bad = bad || (gains.kp_gamma > 200 && gains.kd_gamma < 0.1);
    if (bad) {
        Serial.println("Invalid gains: underdamped");
        return false;
    }
    return true;
}

bool IsValidGaitParams(struct GaitParams params) {
    const float maxL = 0.25;
    const float minL = 0.08;

    float stanceHeight = params.stance_height;
    float downAMP = params.down_amp;
    float upAMP = params.up_amp;
    float flightPercent = params.flight_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    if (stanceHeight + downAMP > maxL || sqrt(pow(stanceHeight, 2) + pow(stepLength / 2.0, 2)) > maxL) {
        Serial.println("Gait overextends leg");
        return false;
    }
    if (stanceHeight - upAMP < minL) {
        Serial.println("Gait underextends leg");
        return false;
    }

    if (flightPercent <= 0 || flightPercent > 1.0) {
        Serial.println("Flight percent is invalid");
        return false;
    }

    if (FREQ < 0) {
        Serial.println("Frequency cannot be negative");
        return false;
    }

    if (FREQ > 10.0) {
        Serial.println("Frequency is too high (>10)");
        return false;
    }

    return true;
}

/**
 * Command the given odrive along a sinusoidal trajectry.
 * TODO finish documentation
 * @param odrive        [description]
 * @param t             [description]
 * @param params        [description]
 * @param gait_offset   [description]
 * @param leg_direction [description]
 * @param gains         [description]
 * @param theta         [description]
 * @param gamma         [description]
 */
void CoupledMoveLeg(ODriveArduino& odrive, float t, struct GaitParams params,
                    float gait_offset, float leg_direction, struct LegGain gains,
                    float& theta, float& gamma) {
    float x; // float x for leg 0 to be set by the sin trajectory
    float y;
    SinTrajectory(t, params, gait_offset, x, y);
    CartesianToThetaGamma(x, y, leg_direction, theta, gamma);
    odrive.SetCoupledPosition(theta, gamma, gains);
}

void gait(struct GaitParams params,
                float leg0_offset, float leg1_offset,
                float leg2_offset, float leg3_offset,
                struct LegGain gains) {

    if (!IsValidGaitParams(params) || !IsValidLegGain(gains)) {
        return;
    }

    float t = millis()/1000.0;

    const float leg0_direction = -1.0;
    CoupledMoveLeg(odrv0Interface, t, params, leg0_offset, leg0_direction, gains,
        global_debug_values.odrv0.sp_theta, global_debug_values.odrv0.sp_gamma);

    const float leg1_direction = -1.0;
    CoupledMoveLeg(odrv1Interface, t, params, leg1_offset, leg1_direction, gains,
        global_debug_values.odrv1.sp_theta, global_debug_values.odrv1.sp_gamma);

    const float leg2_direction = 1.0;
    CoupledMoveLeg(odrv2Interface, t, params, leg2_offset, leg2_direction, gains,
        global_debug_values.odrv2.sp_theta, global_debug_values.odrv2.sp_gamma);

    const float leg3_direction = 1.0;
    CoupledMoveLeg(odrv3Interface, t, params, leg3_offset, leg3_direction, gains,
        global_debug_values.odrv3.sp_theta, global_debug_values.odrv3.sp_gamma);
}

void CommandAllLegs(float theta, float gamma, LegGain gains) {
    odrv0Interface.SetCoupledPosition(theta, gamma, gains);
    odrv1Interface.SetCoupledPosition(theta, gamma, gains);
    odrv2Interface.SetCoupledPosition(theta, gamma, gains);
    odrv3Interface.SetCoupledPosition(theta, gamma, gains);
    global_debug_values.odrv0.sp_theta = theta;
    global_debug_values.odrv0.sp_gamma = gamma;
    global_debug_values.odrv1.sp_theta = theta;
    global_debug_values.odrv1.sp_gamma = gamma;
    global_debug_values.odrv2.sp_theta = theta;
    global_debug_values.odrv2.sp_gamma = gamma;
    global_debug_values.odrv3.sp_theta = theta;
    global_debug_values.odrv3.sp_gamma = gamma;
}

/**
 * Dance gait parameters
 */
void TransitionToDance() {
    state = DANCE;
    Serial.println("DANCE");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    gait_params = {0.15, 0.04, 0.04, 0.75, 0.0, 1.0};
    gait_gains = {120, 0.48, 80, 0.48};
    PrintGaitParams();
}
/**
* Pronk gait parameters
*/
void TransitionToPronk() {
    state = PRONK;
    Serial.println("PRONK");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    gait_params = {0.12, 0.05, 0.0, 0.75, 0.0, 1.0};
    gait_gains = {80, 0.50, 50, 0.50};
    PrintGaitParams();
}

/**
* Bound gait parameters
*/
void TransitionToBound() {
    state = BOUND;
    Serial.println("BOUND");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    gait_params = {0.17, 0.04, 0.06, 0.35, 0.0, 2.0};
    gait_gains = {80, 0.5, 50, 0.5};
    PrintGaitParams();
}

/**
 * Walk gait parameters
 */
void TransitionToWalk() {
    state = WALK;
    Serial.println("WALK");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    gait_params = {0.15, 0.00, 0.06, 0.25, 0.0, 1.5};
    gait_gains = {80, 0.5, 50, 0.5};
    PrintGaitParams();
}
/**
* Trot gait parameters
*/
void TransitionToTrot() {
    state = TROT;
    Serial.println("TROT");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    gait_params = {0.17, 0.04, 0.06, 0.35, 0.15, 2.0};
    gait_gains = {80, 0.5, 50, 0.5};
    PrintGaitParams();
}

void TransitionToRotate() {
    state = ROTATE;
    rotate_start = millis();
    Serial.println("ROTATE");
    gait_gains = {30,0.5,30,0.5};
}
void TransitionToHop() {
    state = HOP;
    Serial.println("HOP");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    gait_params = {0.15, 0.05, 0.05, 0.2, 0, 1.0};
    PrintGaitParams();
}

void test() {
    /* Downwards force test */
    // struct LegGain gains = {0.0, 0.0, 40.0, 0.5};
    // odrv0Interface.SetCoupledPosition(0, PI/3.0f, gains);
    // odrv0Interface.ReadCurrents();

    /* Upwards weight test */
    // struct LegGain gains = {0.0, 0.0, 40.0, 0.5};
    // odrv0Interface.SetCoupledPosition(0, 2.0f*PI/3.0f, gains);
    // odrv0Interface.ReadCurrents();

    /* Sinusoidal upwards force test */
    float low = 20.0f; // corresponds to 5.23A if error is pi/6
    float high = 80.0f; // corresponds to 20.94A if error is pi/6
    float mid = (low + high)/2.0f;
    float amp = high - mid;
    struct LegGain gains = {0.0, 0.0, low + amp * ((int)(millis()/2000) % 2), 0.5};
    odrv0Interface.SetCoupledPosition(0, 2.0*PI/3.0, gains);
    odrv0Interface.ReadCurrents();

}

void hop(struct GaitParams params) {
    float freq = params.freq;
    struct LegGain hop_gains = {120, 1, 80, 1};
    struct LegGain land_gains = {120, 2, 20, 2};
    float theta, gamma;

    CartesianToThetaGamma(0, params.stance_height - params.up_amp, 1, theta, gamma);
    CommandAllLegs(theta, gamma, land_gains);
    chThdSleepMicroseconds(1000000*0.2/freq);

    CartesianToThetaGamma(0, params.stance_height + params.down_amp, 1, theta, gamma);
    CommandAllLegs(theta, gamma, hop_gains);
    chThdSleepMicroseconds(1000000*params.flight_percent/freq);

    CartesianToThetaGamma(0, params.stance_height, 1, theta, gamma);
    CommandAllLegs(theta, gamma, land_gains);
    chThdSleepMicroseconds(1000000*(0.8-params.flight_percent)/freq);


}

void PrintGaitParams() {
    Serial << ("(f)req: ") << gait_params.freq << '\n';
    Serial << ("step (l)ength: ") << gait_params.step_length << '\n';
    Serial << ("stance (h)eight: ") << gait_params.stance_height << '\n';
    Serial << ("(d)own amplitude: ") << gait_params.down_amp << '\n';
    Serial << "(u)p amplitude: " << gait_params.up_amp << '\n';
    Serial << ("flight (p)roportion: ") << gait_params.flight_percent << '\n';
    Serial << "Theta: " << gait_gains.kp_theta << " " << gait_gains.kd_theta << '\n';
    Serial << "Gamma: " << gait_gains.kp_gamma << " " << gait_gains.kd_gamma << '\n';
}
