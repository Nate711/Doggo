#ifndef DATALOG_H
#define DATALOG_H

#include "arduino.h"
#include "datalog.h"
#include "ChRt.h"
#include "SdFat.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "config.h"

// Initialize SD card pin, file on card, and IMU Project
File file;
SdFatSdio sd;

char fileName;
BNO080 myIMU;

THD_WORKING_AREA(waDatalogThread, 128);

THD_FUNCTION(DatalogThread, arg) {
    (void)arg;

    Serial.begin(115200);
    Serial.println("Initializing SD card...");

    //check card is present and can be initialized
    if(!sd.begin()) {
        sd.initErrorHalt();
        Serial.println("Failed");
        return;
    }
    Serial.println("Initialized");


    //Open the file on SD card (only one open at a time)
    // using BNO080 library
    char fileName[13] = "LOGGER00.CSV";

    // Create a new file in the current working directory
    // and generate a new name if the file already exists
    for (uint8_t i = 0; i < 50; i++)
    {
        fileName[6] = i/10 + '0';
        fileName[7] = i%10 + '0';
        if (sd.exists(fileName)) {
            continue;
        } else {
            if(!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
                Serial.println("file.open error");
            }
        }
        Serial.println(fileName);
        break;
    }

    //Initialize IMU data
    Serial.println("BNO080 Read");

    Wire.begin();
    Wire.setClock(400000); //Increase I2C data rate to 400kHz
    myIMU.begin();
    myIMU.enableRotationVector(50); //Send data update every 50ms

    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));

    int count = 0;

    while(true) {
        //counter for flush command
        count ++;
        //long then = micros();

        //Initialize string
        String dataString = "";

        //Read IMU DATA
        if (myIMU.dataAvailable() == true)
        {
            Serial.println("Step1");
            //Get quaternion data
            float quatI = myIMU.getQuatI();
            float quatJ = myIMU.getQuatJ();
            float quatK = myIMU.getQuatK();
            float quatReal = myIMU.getQuatReal();
            float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

            //Transform Quat to Euler angles
            // roll (x-axis rotation)
            double sinr = +2.0 * (quatReal * quatI + quatJ * quatK);
            double cosr = +1.0 - 2.0 * (quatI * quatI + quatJ * quatJ);
            float roll = atan2(sinr, cosr);

            // pitch (y-axis rotation)
            double sinp = +2.0 * (quatReal * quatJ - quatK * quatI);
            float pitch;
            if (fabs(sinp) >= 1) {
                pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            } else {
                pitch = asin(sinp);
            }
            Serial.println("Step2");
            // yaw (z-axis rotation)
            double siny = +2.0 * (quatReal * quatK + quatI * quatJ);
            double cosy = +1.0 - 2.0 * (quatJ * quatJ + quatK * quatK);
            float yaw = atan2(siny, cosy);

            dataString += String(roll) + ", " + String(pitch) + ", " + String(yaw);

            Serial.println(quatRadianAccuracy);


            //Print to Serial monitor
            Serial.println(dataString);

            Serial.println("Step3");
            //If the file is available, write to it
            if (!file.sync() || file.getWriteError()) {
                Serial.println("write error");
            }
            file.println(dataString);

            Serial.println("Step4");
        }

        //long now = micros();
        chThdSleepMicroseconds(1000000/DATALOG_FREQ);

    }
}

#endif
