#include "haptics.h"
#include "position_control.h"

const int SEQUENCE_LENGTH = DEBUG_PRINT_FREQ;
float forces[10][SEQUENCE_LENGTH];

float means[10];
float devs[10];

int currentIndex = 0;
int resetIndex = 0;

void invertMatrix(float mat[2][2]) {
    float a = mat[0][0];
    float b = mat[0][1];
    float c = mat[1][0];
    float d = mat[1][1];
    float invDet = 1.0 / (a * d - b * c);
    mat[0][0] = d * invDet;
    mat[0][1] = -b * invDet;
    mat[1][0] = -c * invDet;
    mat[1][1] = a * invDet;
}

void calculateForce(struct ODrive odrv, float direction, float* xForce, float* yForce) {
    float gamma = odrv.est_gamma;
    float theta = odrv.est_theta;

    float L1 = 0.09F;
    float L2 = 0.162F;
    float k = 0.0245F * 3;

    float L = L1*cos(gamma) + sqrt(L2*L2 - L1*L1*sin(gamma)*sin(gamma));
    float dtheta_dm0 = -0.5F;
    float dtheta_dm1 = -0.5F;
    float dgamma_dm0 = -0.5F;
    float dgamma_dm1 = 0.5F;
    float dradius_dgamma = -L1*sin(gamma) - (L1*L1*sin(gamma)*cos(gamma))/(sqrt(L2*L2 - L1*L1*sin(gamma)*sin(gamma)));
    float dx_dtheta = L*cos(theta);
    float dy_dtheta = -L*sin(theta);
    float dx_dradius = sin(theta);
    float dx_dgamma = dx_dradius * dradius_dgamma;
    float dy_dradius = cos(theta);
    float dy_dgamma = dy_dradius * dradius_dgamma;

    float J[2][2] = {
        {dx_dtheta*dtheta_dm0+dx_dgamma*dgamma_dm0, dx_dtheta*dtheta_dm1+dx_dgamma*dgamma_dm1},
        {dy_dtheta*dtheta_dm0+dy_dgamma*dgamma_dm0, dy_dtheta*dtheta_dm1+dy_dgamma*dgamma_dm1}
    };
    float JT[2][2] = {
        {J[0][0], J[1][0]},
        {J[0][1], J[1][1]}
    };
    invertMatrix(JT);
    float tau0 = odrv.current_0 * k * direction;
    float tau1 = odrv.current_1 * k * direction;
    *xForce = JT[0][0] * tau0 + JT[0][1] * tau1;
    *yForce = JT[1][0] * tau0 + JT[1][1] * tau1;
}

void getAllEvents(ODrive * odrvs[4], float events[10]) {
    float tempX, tempY;
    float totalX = 0.0; 
    float totalY = 0.0;
    for (int i = 0; i < 4; i++) {
        float direction = (i < 2 ? -1 : 1);
        calculateForce(*odrvs[i], direction, &tempX, &tempY);
        getEvent(i * 2, tempX, events);
        getEvent(i * 2 + 1, tempY, events);
        totalX += direction * tempX;
        totalY += tempY;
    }
    getEvent(8, totalX, events);
    getEvent(9, totalY, events);
    currentIndex++;
    if (currentIndex == SEQUENCE_LENGTH)
        currentIndex = 0;
    if (currentIndex == resetIndex)
        resetIndex = -1;
    // Serial.println();
}

void getEvent(int eventIndex, float force, float events[10]) {
    // Serial.print(force);
    // Serial.print(",");
    // Serial.print(means[eventIndex]);
    // Serial.print(",");
    if (isnan(force)) force = 0;
    events[eventIndex] = (force - means[eventIndex]) / (devs[eventIndex] + 1.0);
    if (resetIndex != -1) events[eventIndex] = 0.0;
    means[eventIndex] += (force - forces[eventIndex][currentIndex]) / SEQUENCE_LENGTH;
    forces[eventIndex][currentIndex] = force;
    float dev = forces[eventIndex][0] - means[eventIndex];
    for (int i = 1; i < SEQUENCE_LENGTH; i++) {
        float temp = forces[eventIndex][i] - means[eventIndex];
        if (abs(temp) > abs(dev)) {
            dev = temp;
        }
    }
    devs[eventIndex] = abs(dev);
}

void processEvents(float events[10]) {
    if (resetIndex != -1)
        return;
    float absX = abs(events[8]);
    float absY = abs(events[9]);
    if ((absY > 1.5 || (absX > 1.5 && absX < 1.75)) && state != STOP) {
        TransitionToStop();
        resetEvents();
    } else if (absX > 1.75) {
        TransitionToTrot();
        state_gait_params[state].step_length = (events[8]/absX) * 0.15;
        resetEvents();
    }
}

void resetEvents() {
    resetIndex = currentIndex;
}

void initializeHaptics() {
    for (int i = 0; i < 10; i++) {
        devs[i] = 0;
        means[i] = 0;
        for (int j = 0; j < SEQUENCE_LENGTH; j++) {
            forces[i][j] = 0;
        }
    }
}