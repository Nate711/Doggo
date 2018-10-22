#ifndef DATALOG_H
#define DATALOG_H

#include "arduino.h"
#include "datalog.h"
#include "ChRt.h"
#include "SdFat.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "config.h"
#include "globals.h"

BNO080 bno080_imu;

THD_WORKING_AREA(waIMUThread, 1024);

THD_FUNCTION(IMUThread, arg) {
    (void)arg;

    // Turn on SPI for IMU
    SPI.begin();

    //Initialize IMU
    bno080_imu.beginSPI(SPI_CS_PIN, SPI_WAK_PIN, SPI_INTPIN, SPI_RSTPIN);
    bno080_imu.enableRotationVector(50); //Send data update every 50ms

    if (IMU_VERBOSE > 0) {
        Serial.println("Initializing BNO080...");
        Serial.println(F("Rotation vector enabled at 50Hz"));
    }

    while(true) {

        long read_begin_ts = micros(); //
        //Read IMU DATA
        if (bno080_imu.dataAvailable() == true)
        {
            long read_finished_ts = micros();

            //Get quaternion data
            float quatI = bno080_imu.getQuatI();
            float quatJ = bno080_imu.getQuatJ();
            float quatK = bno080_imu.getQuatK();
            float quatReal = bno080_imu.getQuatReal();
            // float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

            //Transform Quat to Euler angles
            // roll (x-axis rotation)
            double sinr = +2.0 * (quatReal * quatI + quatJ * quatK);
            double cosr = +1.0 - 2.0 * (quatI * quatI + quatJ * quatJ);
            float roll = atan2(sinr, cosr);

            // pitch (y-axis rotation)
            double sinp = +2.0 * (quatReal * quatJ - quatK * quatI);
            float pitch;
            if (fabs(sinp) >= 1) {
                pitch = constrain(pitch, -M_PI, M_PI); // use 90 degrees if out of range
            } else {
                pitch = asin(sinp);
            }

            // yaw (z-axis rotation)
            double cosy = +2.0 * (quatReal * quatK + quatI * quatJ);
            double siny = +1.0 - 2.0 * (quatJ * quatJ + quatK * quatK);
            float yaw = atan2(siny, cosy);

            long imu_calc_done_ts = micros();

            if (IMU_VERBOSE > 0) {
                Serial << "uS spent reading IMU: " << read_finished_ts - read_begin_ts << "\n";
                Serial << "uS total time for IMU: " << imu_calc_done_ts - read_begin_ts << "\n";
                Serial << "Roll: " << roll << "\tPitch: " << pitch << "\tYaw: " << yaw << "\n";
            }

            // Store euler angles to global
            global_debug_values.imu.yaw = yaw;
            global_debug_values.imu.pitch = pitch;
            global_debug_values.imu.roll = roll;

            // TODO: dataAvailable: 2700uS
            // TODO: Read quat: 250uS
        }

        //long now = micros();
        chThdSleepMicroseconds(1000000/IMU_FREQ);
    }
}

#endif
