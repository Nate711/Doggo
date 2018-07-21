#include "ChRt.h"
#include "Arduino.h"
#include "globals.hpp"
#include "config.hpp"

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

static THD_WORKING_AREA(waUSBSerialThread, 128);

static THD_FUNCTION(USBSerialThread, arg) {
    (void)arg;

    while(true) {
      if(Serial.available()) {
          String msg = Serial.readStringUntil('/n');
          // TODO interpret messages
      }

      chThdSleepMicroseconds(1000000/USB_SERIAL_FREQ);
    }
}

#endif
