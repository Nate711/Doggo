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

THD_WORKING_AREA(waIMUThread, 4096);

THD_FUNCTION(IMUThread, arg) {
    (void)arg;

    // Turn on SPI for IMU
    SPI.begin();

    //Initialize IMU
    int polling_period = 1000.0/IMU_SEND_FREQ;
    bno080_imu.beginSPI(SPI_CS_PIN, SPI_WAK_PIN, SPI_INTPIN, SPI_RSTPIN);
    // bno080_imu.enableRotationVector(polling_period); //Send data update every 10ms
    bno080_imu.enableGyro(polling_period);
    bno080_imu.enableAccelerometer(polling_period);

    if (IMU_VERBOSE > 0) {
        Serial.println("Initializing BNO080...");
        Serial.println(F("Rotation vector enabled at 100Hz"));
    }

    float raw_integrated_gyro_y = 0;
    float pitch_estimate = 0;
    float pitch_acc = 0;
    float prev_pitch_acc = 0;
    float rotations = 0;

    while(true) {
        long read_begin_ts = micros(); //
        //Read IMU DATA
        while (bno080_imu.dataAvailable() == true)
        {
            long read_finished_ts = micros();

            // Grab readings from the IMU object
            float gyroY = bno080_imu.getGyroY();
            float accelX = bno080_imu.getAccelX();
            float accelZ = bno080_imu.getAccelZ();

            // Calculate pitch from acceleration data
            pitch_acc = -atan2(accelX, accelZ);

            // Handle multi rotations
            if (pitch_acc - prev_pitch_acc > 0.5*M_PI) {
                rotations -= 1;
            }
            if (pitch_acc - prev_pitch_acc < -0.5*M_PI) {
                rotations += 1;
            }
            float multi_rot_pitch_acc = 2*M_PI*rotations + pitch_acc;

            // Integrate pitch angular rates
            pitch_estimate += gyroY / (float)IMU_FREQ * 2.0;
            raw_integrated_gyro_y += gyroY / (float)IMU_FREQ * 2.0;

            // Apply complementary filter
            float tau = 0.99;
            pitch_estimate = tau * pitch_estimate + (1-tau) * multi_rot_pitch_acc;

            prev_pitch_acc = pitch_acc;

            long imu_calc_done_ts = micros();

            if (IMU_VERBOSE > 0) {
                // Serial << yaw << "\t" << pitch << "\t" << roll << "\n";
                // Serial << quatI << "\t" << quatJ << "\t" << quatK << "\t" << quatReal << "\n";
                Serial << pitch_acc << "\t" << raw_integrated_gyro_y << "\t" << pitch_estimate << "\t" << rotations << "\n";

            }
            if (IMU_VERBOSE > 1) {
                Serial << "uS spent reading IMU: " << read_finished_ts - read_begin_ts << "\n";
                Serial << "uS total time for IMU: " << imu_calc_done_ts - read_begin_ts << "\n";
            }

            // Store euler angles to global
            // global_debug_values.imu.yaw = yaw;
            global_debug_values.imu.pitch = pitch_estimate;
            // global_debug_values.imu.roll = roll;

            // TODO: dataAvailable: 2700uS
            // TODO: Read quat: 250uS
        }

        //long now = micros();
        chThdSleepMicroseconds(1000000/IMU_FREQ);
    }
}

#endif
