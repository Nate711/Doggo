#include "arduino.h"
#include "datalog.h"
#include "ChRt.h"
#include "SdFat.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "config.h"
#include "globals.h"

BNO080 bno080_imu;
float raw_integrated_gyro_y = 0;

THD_WORKING_AREA(waIMUThread, 4096);

THD_FUNCTION(IMUThread, arg) {
    (void)arg;

    // Turn on SPI for IMU
    SPI.begin();

    //Initialize IMU
    int polling_period = 1000.0/IMU_SEND_FREQ;
    if(!bno080_imu.beginSPI(SPI_CS_PIN, SPI_WAK_PIN, SPI_INTPIN, SPI_RSTPIN)) {
        Serial.println("IMU beginSPI failed (non-fatal)...");
        // return;
    }
    bno080_imu.enableGyro(polling_period);

    // Enable accelerometer according to config
    int sensor_count = 1;
    if (IMU_ENABLE_COMPLEMENTARY_FILTER) {
        bno080_imu.enableAccelerometer(polling_period);
        sensor_count = 2;
    }

    if (IMU_VERBOSE > 0) {
        Serial << "Initialized BNO080...\n";
        Serial << "Rotation vector enabled at " << IMU_SEND_FREQ << "Hz\n";
    }

    float pitch_estimate = 0;
    float pitch_acc = 0;
    float prev_pitch_acc = 0;
    float rotations = 0;

    float velocity_x = 0;

    // The BNO080 will be sending both gyro and accel readings every poll period.
    // if we enable the complementary filter.
    // dataAvailable doesn't tell us which sensor was just read so we keep a
    // counter telling us how many readings we got since the last full read.
    int sensors_read = 0;

    while(true) {
        long read_begin_ts = micros(); // Time stamp for before we started reading
        while (bno080_imu.dataAvailable() == true)
        {
            // This block is triggered at a rate of 2*IMU_SEND_FREQ
            sensors_read++;
            if (sensors_read >= sensor_count) {
                sensors_read = 0;
                // This block will execute at the rate of IMU_SEND_FREQ

                long read_finished_ts = micros();

                if (IMU_ENABLE_COMPLEMENTARY_FILTER) {
                    // Grab readings from the IMU object
                    float gyroY = bno080_imu.getGyroY();
                    float accelX = bno080_imu.getAccelX();
                    float accelZ = bno080_imu.getAccelZ();

                    // Calculate pitch from acceleration data
                    pitch_acc = atan2(accelX, accelZ);

                    // Handle multi rotations
                    if (pitch_acc - prev_pitch_acc > 0.5*M_PI) {
                        rotations -= 1;
                    }
                    if (pitch_acc - prev_pitch_acc < -0.5*M_PI) {
                        rotations += 1;
                    }
                    float multi_rot_pitch_acc = 2*M_PI*rotations + pitch_acc;

                    // Integrate pitch angular rates
                    pitch_estimate -= gyroY / (float)IMU_SEND_FREQ;
                    raw_integrated_gyro_y -= gyroY*cos(pitch_estimate) / (float)IMU_SEND_FREQ;

                    velocity_x += (accelX + 9.81*sin(pitch_estimate)) / (float)IMU_SEND_FREQ;

                    // Apply complementary filter
                    float tau = IMU_COMPLEMENTARY_FILTER_TAU;
                    pitch_estimate = tau * pitch_estimate + (1-tau) * multi_rot_pitch_acc;

                    prev_pitch_acc = pitch_acc;

                    long imu_calc_done_ts = micros();
                    if (IMU_VERBOSE > 0) {
                        Serial << pitch_acc << "\t" << raw_integrated_gyro_y << "\t" << pitch_estimate << "\t" << rotations << "\t" << velocity_x << "\n";

                    }
                    if (IMU_VERBOSE > 1) {
                        Serial << "uS spent reading IMU: " << read_finished_ts - read_begin_ts << "\n";
                        Serial << "uS total time for IMU: " << imu_calc_done_ts - read_begin_ts << "\n";
                    }

                    // Store euler angles to global variable
                    global_debug_values.imu.pitch = pitch_estimate;
                } else {
                    float gyroY = bno080_imu.getGyroY();
                    raw_integrated_gyro_y -= gyroY / (float)IMU_SEND_FREQ;
                    long imu_calc_done_ts = micros();

                    if (IMU_VERBOSE > 0) {
                        Serial << raw_integrated_gyro_y << "\n";

                    }
                    if (IMU_VERBOSE > 1) {
                        Serial << "uS spent reading IMU: " << read_finished_ts - read_begin_ts << "\n";
                        Serial << "uS total time for IMU: " << imu_calc_done_ts - read_begin_ts << "\n";
                    }

                    // Store euler angles to global variable
                    global_debug_values.imu.pitch = raw_integrated_gyro_y;
                }

                // TODO: dataAvailable:
                // TODO: Read gyro and accel:
            }
        }

        chThdSleepMicroseconds(1000000/IMU_FREQ);
    }
}

void IMUTarePitch() {
    raw_integrated_gyro_y = 0;
    global_debug_values.imu.pitch = 0;
    if (IMU_VERBOSE > 0) {
        Serial << "Zero-ed body pitch\n";
    }
}
