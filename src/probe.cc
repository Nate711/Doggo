#include "probe.h"
#include "ODriveArduino.h"
#include "globals.h"
#include "position_control.h"

// Privates
long probing_timestamp;
bool first_probe = true;

float last_bounds_check = 0;

void StartProbe() {
    probing_timestamp = millis();
    struct LegGains probe_gains_init{.2, .002, .02, .001};
    // TODO: Call function to Set gains
    Serial.println("PD gains set.");
	Serial.println("Probing initialized.");
    state = PROBE;
}

// TODO: Replace Dual Vesc Terms with ODrive 
void ProbingControl(float t, float& theta_sp){
	float theta_amp = 45.0;
	float theta_offset = 45.0;
	float freq = 0.0005; //
	theta_sp = sinusoid(t, theta_amp, freq, 0.0 - 1.57, theta_offset);
}

// TODO: Replace Dual Vesc Terms with ODrive
void Probe() {
	float millis_probing = millis() - probing_timestamp;
		//if(millis_probing >= 2000) probing_timestamp = millis();
		float theta_sp;
		float gamma_sp = 45.0;
		ProbingControl(millis_probing, theta_sp);

		if(first_probe){
			first_probe = false;
			StartProbe();
		} else {
			if(abs(dual_vesc.get_theta() - last_theta) < 1) {
				gamma_sp = dual_vesc.get_gamma() - 5;
			}
			if (millis() % 5000 == 0) {
				last_theta = dual_vesc.get_theta();
			}

			dual_vesc.pid_update(theta_sp, gamma_sp);
		}

		// TODO: Move touch delay implementation outside Probe into loop
		int touch_delay = 500;

			if((abs(ia) > thres || abs(ib) > thres) && (millis_running > touch_delay)) {
				Serial.println(millis_running);
				transition_to_ESTOP();
			}
}