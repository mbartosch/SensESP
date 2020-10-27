// VE.Direct device example

#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// #define SERIAL_DEBUG_DISABLED

#define USE_LIB_WEBSOCKET true

#include "sensesp_app.h"
#include "sensors/vedirect.h"
#include "signalk/signalk_output.h"

ReactESP app([]() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // set up VE.Direct connection via software serial port
  const byte VictronSerialRXPin = 12; // D6 - GPIO12 - RX
  const byte VictronSerialTXPin = -1; // not used

  SoftwareSerial* VictronSerial = new SoftwareSerial(VictronSerialRXPin, VictronSerialTXPin);
  VictronSerial->begin(19200);

  // Attach VE.Direct Parser to serial interface
  VEDirectInput* vedirect = new VEDirectInput(VictronSerial);

  // and enable the parser
  vedirect->enable();

  // for now without SensESP...
  // sensesp_app = new SensESPApp();
  // sensesp_app->enable();
});
