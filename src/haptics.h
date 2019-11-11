#ifndef HAPTICS_H
#define HAPTICS_H
#include "globals.h"
#include "config.h"

void invertMatrix(float m[2][2]);
void calcForces(struct ODrive odrv, float direction, float* xForce, float* yForce);
void getAllEvents(ODrive * odrvs[4], float events[10]);
void getEvent(int eventIndex, float force, float events[10]);
void processEvents(float events[10]);
void resetEvents();
void initializeHaptics();

#endif