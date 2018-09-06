#ifndef PROBE_H
#define PROBE_H

#include "ODriveArduino.h"

void StartProbe();
void ProbingControl(float millis_probing, float& theta_sp);
void Probe();

#endif
