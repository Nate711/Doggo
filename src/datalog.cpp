#ifndef DATALOG_H
#define DATALOG_H

#include "arduino.h"
#include "datalog.h"
#include "ChRt.h"
#include "SdFat.h"
#include "config.h"
#include "globals.h"


// Initialize SD card pin, file on card, and IMU Project
File file;
SdFatSdio sd;

char fileName;

THD_WORKING_AREA(waDatalogThread, 2048);

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
        break;
    }
    Serial.print("Writing to: ");
    Serial.println(fileName);

    while(true) {
        long tic = micros();

        // TODO: Query the attitude data from the imu struct
        float yaw = 0, pitch = 0, roll = 0;
        file.print(yaw);
        file.print(",");
        file.print(pitch);
        file.print(",");
        file.println(roll);

        Serial << "Write: " << micros()-tic << "\n";

        // NOTE: sd write: 5400uS

        chThdSleepMicroseconds(1000000/DATALOG_FREQ);

    }
}

#endif
