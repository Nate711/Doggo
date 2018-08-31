#include "usb_serial.h"
#include "ChRt.h"
#include "Arduino.h"
#include "globals.h"
#include "config.h"
#include "jump.h"

THD_WORKING_AREA(waUSBSerialThread, 128);

THD_FUNCTION(USBSerialThread, arg) {
    (void)arg;

    while(true) {
        if(Serial.available()) {
            char c = Serial.read();
            // TODO interpret messages
            switch(c) {
                case 'j':
                    StartJump(millis()/1000.0f);
                    Serial.println("Jump");
                    break;
                default:
                    Serial.println("Unknown command");
                break;
            }
        }

        chThdSleepMicroseconds(1000000/USB_SERIAL_FREQ);
    }
}
