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

    if (ENABLE_DATALOGGER != 1) {
        return;
    }

    Serial.begin(115200);

    //check card is present and can be initialized
    if(!sd.begin()) {
        sd.initErrorHalt();
        Serial.println("Failed to initialize SD Card");
        return;
    }
    if (DATALOGGER_VERBOSE > 0) {
        Serial.println("Initialized SD Card");
    }

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
    if (DATALOGGER_VERBOSE > 0) {
        Serial << ("Writing to file: ") << fileName << "\n";
    }

    while(true) {
        long tic = micros();

        // TODO: Query the attitude data from the imu struct
        file.print(global_debug_values.imu.yaw);
        file.print(",");
        file.print(global_debug_values.imu.pitch);
        file.print(",");
        file.println(global_debug_values.imu.roll);

        if (DATALOGGER_VERBOSE > 0) {
            Serial << "Write took: " << micros()-tic << "us\n";
        }

        // TODO: sd write takes [x] us

        chThdSleepMicroseconds(1000000/DATALOG_FREQ);
    }
}

#endif
