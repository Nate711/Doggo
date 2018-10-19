#ifndef DATALOG_H
#define DATALOG_H

#include "arduino.h"
#include "datalog.h"
#include "ChRt.h"
#include "SdFat.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "config.h"
#include "globals.h"

BNO080 myIMU;

THD_WORKING_AREA(waIMUThread, 1924);

THD_FUNCTION(IMUThread, arg) {
    (void)arg;

    // Turn on SPI for IMU
    SPI.begin();

    //Initialize IMU
    Serial.println("Initializing BNO080...");

    myIMU.beginSPI(SPI_CS_PIN, SPI_WAK_PIN, SPI_INTPIN, SPI_RSTPIN);
    myIMU.enableRotationVector(50); //Send data update every 50ms

    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));

    while(true) {

        //Initialize string
        String dataString = "";

        long avail = micros();
        //Read IMU DATA
        if (myIMU.dataAvailable() == true)
        {
            long tic = micros();
            Serial << "DataAvailable Time: " << tic - avail << "\n";

            //Get quaternion data
            float quatI = myIMU.getQuatI();
            float quatJ = myIMU.getQuatJ();
            float quatK = myIMU.getQuatK();
            float quatReal = myIMU.getQuatReal();
            float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
            long tic0 = micros();

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

            long toc = micros();
            Serial << "Elapsed: " << toc-tic << "\n";
            Serial << "Read Quat: " << tic0-tic << "\n";

            Serial << "Roll: " << roll << "\tPitch: " << pitch << "\tYaw: " << yaw << "\n";

            Serial.println("Wrote Data");

            // TODO: dataAvailable: 2700uS
            // TODO: Read quat: 250uS
        }

        //long now = micros();
        chThdSleepMicroseconds(1000000/IMU_FREQ);
    }
}

#endif
