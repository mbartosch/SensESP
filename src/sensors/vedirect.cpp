// SignalK sensor class for passively reading data from a Victron
// BMV600, BMV700, MPPT or Phoenix device.
// 
// 2020-10-26 Martin Bartosch for SY Entropy

#include "vedirect.h"
#include "sensesp.h"
#include "signalk/signalk_output.h"

// set up a standard sensor for a BMV device (common to BMV600 and BMV700)
VEDirectInput* setup_vedirect_bmv(Stream* rx_stream, std::string deviceName) {
  VEDirectInput* vedirect = new VEDirectInput(rx_stream, "/vedirect");
  std::string prefix = "electrical.batteries.";
  prefix.append(deviceName);
  
  std::string path;
  
  path = prefix;
  path.append(".voltage");
  vedirect->data.mainVoltage.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".auxvoltage");
  vedirect->data.auxVoltage.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".current");
  vedirect->data.batteryCurrent.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".capacity.dischargeSinceFull");
  vedirect->data.consumedCoulombs.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".capacity.stateOfCharge");
  vedirect->data.stateOfCharge.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".capacity.timeRemaining");
  vedirect->data.timeToGo.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".alarm.state");
  vedirect->data.alarmState.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".relay.state");
  vedirect->data.relayState.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".alarm.reason");
  vedirect->data.alarmReason.connect_to(
    new SKOutputString(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.depthOfDeepestDischarge");
  vedirect->data.depthOfDeepestDischarge.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.depthOfLastDischarge");
  vedirect->data.depthOfLastDischarge.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.depthOfAverageDischarge");
  vedirect->data.depthOfAverageDischarge.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.totalChargeCycles");
  vedirect->data.totalChargeCycles.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.totalFullCycles");
  vedirect->data.totalFullCycles.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.cumulativeCoulombsDrawn");
  vedirect->data.cumulativeCoulombsDrawn.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.minimumMainVoltage");
  vedirect->data.minimumMainVoltage.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.maximumMainVoltage");
  vedirect->data.maximumMainVoltage.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.timeSinceLastFullCharge");
  vedirect->data.timeSinceLastFullCharge.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.totalAutomaticSynchronizations");
  vedirect->data.totalAutomaticSynchronizations.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.totalLowMainVoltageAlarms");
  vedirect->data.totalLowMainVoltageAlarms.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.totalHighMainVoltageAlarms");
  vedirect->data.totalHighMainVoltageAlarms.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.totalHighMainVoltageAlarms");
  vedirect->data.totalHighMainVoltageAlarms.connect_to(
    new SKOutputInt(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.minimumAuxVoltage");
  vedirect->data.minimumAuxVoltage.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".statistics.maximumAuxVoltage");
  vedirect->data.maximumAuxVoltage.connect_to(
    new SKOutputNumber(path.c_str(), ""));

  path = prefix;
  path.append(".info.modelDescription");
  vedirect->data.modelDescription.connect_to(
    new SKOutputString(path.c_str(), ""));

  path = prefix;
  path.append(".info.firmwareVersion");
  vedirect->data.firmwareVersion.connect_to(
    new SKOutputString(path.c_str(), ""));

  return vedirect;
}



VEDirectInput::VEDirectInput(Stream* rx_stream, String config_path, byte signalKUnitTransform)
    : Sensor(config_path), mode_signalKUnitTransform(signalKUnitTransform) {
  this->rx_stream = rx_stream;
 
  this->receiveState = VEDIRECT_RX_STATE_UNKNOWN;
  this->rxBufIndex = 0;
  this->checksum = 0;

  this->clearLabelBuffer();

  byte ii = 0;
  const byte st = VEDIRECT_VALUE_STATE_UNDEFINED; // to make assignments below shorter

  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "V",            1, "V",     "", "", 0, st, &this->data.mainVoltage }; // mV; Main (battery) voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "VS",           1, "V",     "", "", 0, st, &this->data.auxVoltage }; // mV; Auxiliary (starter) voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "VM",           1, "V",     "", "", 0, st, &this->data.midPointVoltage }; // mV; Mid-point voltage of the battery bank
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "DM",           1, "ratio", "", "", 0, st, &this->data.midPointDeviation }; // 1/10 %; Mid-point deviation of the battery bank
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "VPV",          1, "V",     "", "", 0, st, &this->data.panelVoltage }; // mV; Panel voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "PPV",          1, "A",     "", "", 0, st, &this->data.panelPower }; // W; Panel power
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "I",            1, "A",     "", "", 0, st, &this->data.batteryCurrent }; // mA; Battery current
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "IL",           1, "A",     "", "", 0, st, &this->data.loadCurrent }; // mA; Load current
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "LOAD",        -1, "state", "", "", 0, st, &this->data.loadOutputState }; // Load output state (ON/OFF)
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "T",         -273, "K",     "", "", 0, st, &this->data.batteryTemperature }; // °C; Battery temperature
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "P",         1000, "W",     "", "", 0, st, &this->data.instantaneousPower }; // W; Instantaneous power
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "CE",        3600, "C",     "", "", 0, st, &this->data.consumedCoulombs }; // mAh; Consumed Amp Hours
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "SOC",          1, "ratio", "", "", 0, st, &this->data.stateOfCharge }; // 1/10 %; State-of-charge
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "TTG",      60000, "s",     "", "", 0, st, &this->data.timeToGo }; // min; Time-to-go
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "Alarm",       -1, "state", "", "", 0, st, &this->data.alarmState }; // Alarm (ON/OFF)
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "Relay",       -1, "state", "", "", 0, st, &this->data.relayState }; // Relay (ON/OFF)
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "AR",           0, "",      "", "", 0, st, &this->data.alarmReason }; // Alarm reason
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H1",        3600, "C",     "", "", 0, st, &this->data.depthOfDeepestDischarge }; // mAh; Depth of the deepest discharge
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H2",        3600, "C",     "", "", 0, st, &this->data.depthOfLastDischarge }; // mAh; Depth of the last discharge
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H3",        3600, "C",     "", "", 0, st, &this->data.depthOfAverageDischarge }; // mAh; Depth of the average discharge
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H4",          -2, "count", "", "", 0, st, &this->data.totalChargeCycles }; // Number of charge cycles
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H5",          -2, "count", "", "", 0, st, &this->data.totalFullCycles }; // Number of full discharges
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H6",        3600, "C",     "", "", 0, st, &this->data.cumulativeCoulombsDrawn }; // mAh; Cumulative Amp Hours drawn
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H7",           1, "V",     "", "", 0, st, &this->data.minimumMainVoltage }; // mV; Minimum main (battery) voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H8",           1, "V",     "", "", 0, st, &this->data.maximumMainVoltage }; // mV; Maximum main (battery) voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H9",        1000, "s",     "", "", 0, st, &this->data.timeSinceLastFullCharge }; // s; Number of seconds since last full charge
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H10",         -2, "count", "", "", 0, st, &this->data.totalAutomaticSynchronizations }; // Number of automatic synchronizations
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H11",         -2, "count", "", "", 0, st, &this->data.totalLowMainVoltageAlarms }; // Number of low main voltage alarms
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H12",         -2, "count", "", "", 0, st, &this->data.totalHighMainVoltageAlarms }; // Number of high main voltage alarms
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H13",         -2, "count", "", "", 0, st, &this->data.totalLowAuxVoltageAlarms}; // Number of low auxiliary voltage alarms
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H14",         -2, "count", "", "", 0, st, &this->data.totalHighAuxVoltageAlarms }; // Number of high auxiliary voltage alarms
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H15",          1, "V",     "", "", 0, st, &this->data.minimumAuxVoltage }; // mV; Minimum auxiliary (battery) voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H16",          1, "V",     "", "", 0, st, &this->data.maximumAuxVoltage }; // mV; Maximum auxiliary (battery) voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H17",      36000, "J",     "", "", 0, st, &this->data.dischargedEnergy }; // 10 Wh; Amount of discharged energy
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H18",      36000, "J",     "", "", 0, st, &this->data.chargedEnergy }; // 10 Wh; Amount of charged energy
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H19",      36000, "J",     "", "", 0, st, &this->data.totalYield }; // 10 Wh; Yield total (user resettable counter)
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H20",      36000, "J",     "", "", 0, st, &this->data.todaysYield }; // 10 Wh; Yield today
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H21",       1000, "W",     "", "", 0, st, &this->data.todaysMaximumPower }; // W; Maximum power today
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H22",      36000, "J",     "", "", 0, st, &this->data.yesterdaysYield }; // 10 Wh; Yield yesterday
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "H23",       1000, "W",     "", "", 0, st, &this->data.yesterdaysMaximumPower }; // W; Maximum power yesterday
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "ERR",          0, "",      "", "", 0, st, &this->data.errorCode }; // Error code
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "CS",           0, "",      "", "", 0, st, &this->data.stateOfOperation }; // State of operation
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "BMV",          0, "",      "", "", 0, st, &this->data.modelDescription }; // Model description (deprecated)
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "FW",           0, "",      "", "", 0, st, &this->data.firmwareVersion }; // Firmware version
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "PID",          0, "",      "", "", 0, st, &this->data.productID }; // Product ID
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "SER#",         0, "",      "", "", 0, st, &this->data.serialNumber }; // Serial number
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "HSDS",        -2, "",      "", "", 0, st, &this->data.daySequenceNumber }; // Day sequence number (0..364)
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "MODE",         0, "",      "", "", 0, st, &this->data.deviceMode }; // Device mode
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "AC_OUT_V",   100, "V",     "", "", 0, st, &this->data.acOutputVoltage }; // 0.01 V; AC output voltage
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "AC_OUT_I",   100, "V",     "", "", 0, st, &this->data.acOutputCurrent }; // 0.1 A; AC output current
  this->VEDirectFields[ii++] =
    (VEDirectFieldEntry){ "WARN",         0, "",      "", "", 0, st, &this->data.warningReason }; // Warning reason

  load_configuration();
}

 // change SignalK value transform mode
void VEDirectInput::setSignalKUnitTransform(byte mode) {
  mode_signalKUnitTransform = mode;
}

// internal helpers
// clear label buffer and reset index pointer
void VEDirectInput::clearLabelBuffer() {
  for (byte ii = 0; ii < VEDIRECT_BUF_LABEL_SIZE; ii++) 
    this->fieldLabel[ii] = 0;
  this->rxBufIndex = 0;
}

// clear value buffer and reset index pointer
void VEDirectInput::clearValueBuffer() {
  for (byte ii = 0; ii < VEDIRECT_BUF_VALUE_SIZE; ii++) 
    this->fieldValue[ii] = 0;
  this->rxBufIndex = 0;
}

// determine the array index of the VE.Direct entries
byte VEDirectInput::getVEDirectIndex(const char * label) {
  for (byte ii = 0; ii < VEDIRECT_TOTAL_LABELS; ii++) {
    if (strcmp(label, this->VEDirectFields[ii].label) == 0)
      return ii;
  }
  return -1;
}

// process one raw label, value tuple
void VEDirectInput::processField() {
  if (strcmp(this->fieldLabel, "Checksum") == 0) {
    if (this->checksum == 0) {
      this->commit();
    } else {
      // debugD("Checksum INVALID");
    }
    // re-initialize checksum for next block
    this->checksum = 0;
    return; // do not store label "Commit"
  }

  byte index = this->getVEDirectIndex(this->fieldLabel);
  if (index >= 0) {
    // record value as dirty
    this->VEDirectFields[index].status = VEDIRECT_VALUE_STATE_DATA_DIRTY;
    this->VEDirectFields[index].lastupdate = millis();
    strncpy(this->VEDirectFields[index].valuePending, this->fieldValue, VEDIRECT_BUF_VALUE_SIZE);
  } else {
    // debugD("ERROR: invalid label: %s", this->fieldLabel);
  }
}

// propagate valid fields to output buffer
void VEDirectInput::commit() {
  for(byte ii = 0; ii < VEDIRECT_TOTAL_LABELS; ii++) {
    if (this->VEDirectFields[ii].status == VEDIRECT_VALUE_STATE_DATA_DIRTY) {
      this->VEDirectFields[ii].status = VEDIRECT_VALUE_STATE_DATA_VALID;
      if (this->mode_signalKUnitTransform == 0) {
        // literal output, no SignalK unit transformation
        strncpy(VEDirectFields[ii].valueConfirmed, VEDirectFields[ii].valuePending, VEDIRECT_BUF_VALUE_SIZE);
      } else {
        // "---" raw values indicate invalid/irrelevant data, ignore
        if (strcmp(VEDirectFields[ii].valuePending, "---") == 0)
          continue;

        if (this->VEDirectFields[ii].unit_transform_factor > 0) {
          // unit conversion
          float value;
          value = (atof(VEDirectFields[ii].valuePending) * this->VEDirectFields[ii].unit_transform_factor) / 1000;
          sprintf(VEDirectFields[ii].valueConfirmed, "%.3f", value);

          // update SignalK data record
          VEDirectFields[ii].floatValue->set(value);

        } else if (this->VEDirectFields[ii].unit_transform_factor == 0) {
          // literal output
          strncpy(VEDirectFields[ii].valueConfirmed, VEDirectFields[ii].valuePending, VEDIRECT_BUF_VALUE_SIZE);

          // update SignalK data record
          VEDirectFields[ii].stringValue->set(VEDirectFields[ii].valueConfirmed);

        } else if (this->VEDirectFields[ii].unit_transform_factor == -1) {
          // binary state, convert to 0/1
          if (strncmp(VEDirectFields[ii].valuePending, "ON", 2) == 0) {
            strncpy(VEDirectFields[ii].valueConfirmed, "1", VEDIRECT_BUF_VALUE_SIZE);

            // update SignalK data record
            VEDirectFields[ii].intValue->set(1);

          } else if (strncmp(VEDirectFields[ii].valuePending, "OFF", 3) == 0) {
            strncpy(VEDirectFields[ii].valueConfirmed, "0", VEDIRECT_BUF_VALUE_SIZE);

            // update SignalK data record
            VEDirectFields[ii].intValue->set(0);

          } else {
            sprintf(VEDirectFields[ii].valueConfirmed, "invalid: -%s-", VEDirectFields[ii].valuePending);
          }
        } else if (this->VEDirectFields[ii].unit_transform_factor == -2) {
          // int conversion
          int value;
          value = atoi(VEDirectFields[ii].valuePending);
          strncpy(VEDirectFields[ii].valueConfirmed, VEDirectFields[ii].valuePending, VEDIRECT_BUF_VALUE_SIZE);

          // update SignalK data record
          VEDirectFields[ii].intValue->set(value);

        } else if (this->VEDirectFields[ii].unit_transform_factor == -273) {
          // °C to K converstion
          float value;
          value = atof(VEDirectFields[ii].valuePending) + 273.15;
          sprintf(VEDirectFields[ii].valueConfirmed, "%.2f", value);

          // update SignalK data record
          VEDirectFields[ii].floatValue->set(value);

        } else
        // default to literal output (should not happen, don't update sk data)
          strncpy(VEDirectFields[ii].valueConfirmed, VEDirectFields[ii].valuePending, VEDIRECT_BUF_VALUE_SIZE);
      }
    }
  }
  this->notify();
}

// register handlers
void VEDirectInput::enable() {
  // process serial input
  app.onAvailable(*rx_stream, [this]() {
    while (this->rx_stream->available() && this->receiveState != VEDIRECT_RX_STATE_RECORDCOMPLETE) {
        char c = this->rx_stream->read();
        this->checksum += c;

        switch (this->receiveState) {
          case VEDIRECT_RX_STATE_LABEL:
            if (c != '\t') {
              this->fieldLabel[this->rxBufIndex++] = c;
            } else {
              // end of field
              this->receiveState = VEDIRECT_RX_STATE_VALUE;
              this->clearValueBuffer();
            }
            break;
          case VEDIRECT_RX_STATE_VALUE:
            if (c != '\n') {
              // string values may contain \r\n instead of the documented \n, ignore the return
              if (c != '\r')
                this->fieldValue[this->rxBufIndex] = c;
              this->rxBufIndex++;
            } else {
              // end of field
              this->receiveState = VEDIRECT_RX_STATE_RECORDCOMPLETE;
            }
            break;
          case VEDIRECT_RX_STATE_UNKNOWN:
            // fall-through intended
          default:
            if (c == '\n') {
              // synchronize state after end of record
              this->receiveState = VEDIRECT_RX_STATE_LABEL;
              this->clearLabelBuffer();
            }
        }
    }

    if (this->receiveState == VEDIRECT_RX_STATE_RECORDCOMPLETE) {
        this->processField();
        this->receiveState = VEDIRECT_RX_STATE_LABEL;
        this->clearLabelBuffer();
    }
  });


  #ifndef DEBUG_DISABLED
  #if 0
  app.onRepeat(5000, [this](){
    debugD("VE.Direct data");
    for (byte ii = 0; ii < VEDIRECT_TOTAL_LABELS; ii++) {
      // dump all data fields ever seen from this device
      if (this->VEDirectFields[ii].status != VEDIRECT_VALUE_STATE_UNDEFINED) {
        debugD("%s: %s (%lu ms)", this->VEDirectFields[ii].label, this->VEDirectFields[ii].valueConfirmed, millis() - this->VEDirectFields[ii].lastupdate);
      }
    }
  });
  #endif
  #endif
}

