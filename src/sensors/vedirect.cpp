// SignalK sensor class for passively reading data from a Victron
// BMV600, BMV700, MPPT or Phoenix device.
// 
// 2020-10-26 Martin Bartosch for SY Entropy

#include "vedirect.h"
#include "sensesp.h"

VEDirectInput::VEDirectInput(Stream* rx_stream, String config_path, byte signalKUnitTransform)
    : Sensor(config_path), mode_signalKUnitTransform(signalKUnitTransform) {
  this->rx_stream = rx_stream;
 
  this->receiveState = VEDIRECT_RX_STATE_UNKNOWN;
  this->rxBufIndex = 0;
  this->checksum = 0;

  this->clearLabelBuffer();

  byte ii = 0;
  const byte st = VEDIRECT_VALUE_STATE_UNDEFINED; // to make assignments below shorter

  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "V",            1, "V",     "", "", 0, st }; // mV; Main (battery) voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "VS",           1, "V",     "", "", 0, st }; // mV; Auxiliary (starter) voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "VM",           1, "V",     "", "", 0, st }; // mV; Mid-point voltage of the battery bank
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "DM",           1, "ratio", "", "", 0, st }; // 1/10 %; Mid-point deviation of the battery bank
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "VPV",          1, "V",     "", "", 0, st }; // mV; Panel voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "PPV",          1, "A",     "", "", 0, st }; // W; Panel power
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "I",            1, "A",     "", "", 0, st }; // mA; Battery current
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "IL",           1, "A",     "", "", 0, st }; // mA; Load current
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "LOAD",        -1, "state", "", "", 0, st }; // Load output state (ON/OFF)
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "T",         -273, "K",     "", "", 0, st }; // °C; Battery temperature
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "P",         1000, "W",     "", "", 0, st }; // W; Instantaneous power
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "CE",        3600, "C",     "", "", 0, st }; // mAh; Consumed Amp Hours
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "SOC",          1, "ratio", "", "", 0, st }; // 1/10 %; State-of-charge
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "TTG",      60000, "s",     "", "", 0, st }; // min; Time-to-go
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "Alarm",       -1, "state", "", "", 0, st }; // Alarm (ON/OFF)
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "Relay",       -1, "state", "", "", 0, st }; // Relay (ON/OFF)
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "AR",           0, "",      "", "", 0, st }; // Alarm reason
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H1",        3600, "C",     "", "", 0, st }; // mAh; Depth of the deepest discharge
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H2",        3600, "C",     "", "", 0, st }; // mAh; Depth of the last discharge
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H3",        3600, "C",     "", "", 0, st }; // mAh; Depth of the average discharge
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H4",        1000, "count", "", "", 0, st }; // Number of charge cycles
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H5",        1000, "count", "", "", 0, st }; // Number of full discharges
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H6",        3600, "C",     "", "", 0, st }; // mAh; Cumulative Amp Hours drawn
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H7",           1, "V",     "", "", 0, st }; // mV; Minimum main (battery) voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H8",           1, "V",     "", "", 0, st }; // mV; Maximum main (battery) voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H9",        1000, "s",     "", "", 0, st }; // s; Number of seconds since last full charge
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H10",       1000, "count", "", "", 0, st }; // Number of automatic synchronizations
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H11",       1000, "count", "", "", 0, st }; // Number of low main voltage alarms
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H12",       1000, "count", "", "", 0, st }; // Number of high main voltage alarms
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H13",       1000, "count", "", "", 0, st }; // Number of low auxiliary voltage alarms
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H14",       1000, "count", "", "", 0, st }; // Number of high auxiliary voltage alarms
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H15",          1, "V",     "", "", 0, st }; // mV; Minimum auxiliary (battery) voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H16",          1, "V",     "", "", 0, st }; // mV; Maximum auxiliary (battery) voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H17",      36000, "J",     "", "", 0, st }; // 10 Wh; Amount of discharged energy
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H18",      36000, "J",     "", "", 0, st }; // 10 Wh; Amount of charged energy
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H19",      36000, "J",     "", "", 0, st }; // 10 Wh; Yield total (user resettable counter)
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H20",      36000, "J",     "", "", 0, st }; // 10 Wh; Yield today
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H21",       1000, "W",     "", "", 0, st }; // W; Maximum power today
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H22",      36000, "J",     "", "", 0, st }; // 10 Wh; Yield yesterday
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "H23",       1000, "W",     "", "", 0, st }; // W; Maximum power yesterday
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "ERR",           0, "",      "", "", 0, st }; // Error code
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "CS",            0, "",      "", "", 0, st }; // State of operation
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "BMV",          0, "",      "", "", 0, st }; // Model description (deprecated)
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "FW",           0, "",      "", "", 0, st }; // Firmware version
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "PID",          0, "",      "", "", 0, st }; // Product ID
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "SER#",         0, "",      "", "", 0, st }; // Serial number
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "HSDS",         0, "",      "", "", 0, st }; // Day sequence number (0..364)
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "MODE",         0, "",      "", "", 0, st }; // Device mode
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "AC_OUT_V",   100, "V",     "", "", 0, st }; // 0.01 V; AC output voltage
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "AC_OUT_I",   100, "V",     "", "", 0, st }; // 0.1 A; AC output current
  this->VEDirectData[ii++] =
    (VEDirectFieldEntry){ "WARN",      1000, "",      "", "", 0, st }; // Warning reason
  
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
    if (strcmp(label, this->VEDirectData[ii].label) == 0)
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
    this->VEDirectData[index].status = VEDIRECT_VALUE_STATE_DATA_DIRTY;
    this->VEDirectData[index].lastupdate = millis();
    strncpy(this->VEDirectData[index].valuePending, this->fieldValue, VEDIRECT_BUF_VALUE_SIZE);
  } else {
    // debugD("ERROR: invalid label: %s", this->fieldLabel);
  }
}

// propagate valid fields to output buffer
void VEDirectInput::commit() {
  for(byte ii = 0; ii < VEDIRECT_TOTAL_LABELS; ii++) {
    if (this->VEDirectData[ii].status == VEDIRECT_VALUE_STATE_DATA_DIRTY) {
      this->VEDirectData[ii].status = VEDIRECT_VALUE_STATE_DATA_VALID;
      if (this->mode_signalKUnitTransform == 0) {
        // literal output, no SignalK unit transformation
        strncpy(VEDirectData[ii].valueConfirmed, VEDirectData[ii].valuePending, VEDIRECT_BUF_VALUE_SIZE);
      } else {
        // "---" raw values indicate invalid/irrelevant data, ignore
        if (strcmp(VEDirectData[ii].valuePending, "---") == 0)
          continue;

        if (this->VEDirectData[ii].unit_transform_factor > 0) {
          // unit conversion
          float value;
          value = (atof(VEDirectData[ii].valuePending) * this->VEDirectData[ii].unit_transform_factor) / 1000;
          sprintf(VEDirectData[ii].valueConfirmed, "%.3f", value);
        } else if (this->VEDirectData[ii].unit_transform_factor == 0) {
          // literal output
          strncpy(VEDirectData[ii].valueConfirmed, VEDirectData[ii].valuePending, VEDIRECT_BUF_VALUE_SIZE);
        } else if (this->VEDirectData[ii].unit_transform_factor == -1) {
          // binary state, convert to 0/1
          if (strncmp(VEDirectData[ii].valuePending, "ON", 2) == 0) {
            strncpy(VEDirectData[ii].valueConfirmed, "1", VEDIRECT_BUF_VALUE_SIZE);
          } else if (strncmp(VEDirectData[ii].valuePending, "OFF", 3) == 0) {
            strncpy(VEDirectData[ii].valueConfirmed, "0", VEDIRECT_BUF_VALUE_SIZE);
          } else {
            sprintf(VEDirectData[ii].valueConfirmed, "invalid: -%s-", VEDirectData[ii].valuePending);
          }
        } else if (this->VEDirectData[ii].unit_transform_factor == -273) {
          // °C to K converstion
          float value;
          value = atof(VEDirectData[ii].valuePending) + 273.15;
          sprintf(VEDirectData[ii].valueConfirmed, "%.2f", value);
        } else
        // default to literal output (should not happen)
          strncpy(VEDirectData[ii].valueConfirmed, VEDirectData[ii].valuePending, VEDIRECT_BUF_VALUE_SIZE);
      }
    }
  }
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
              this->fieldValue[this->rxBufIndex++] = c;
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
  app.onRepeat(5000, [this](){
    debugD("VE.Direct data");
    for (byte ii = 0; ii < VEDIRECT_TOTAL_LABELS; ii++) {
      // dump all data fields ever seen from this device
      if (this->VEDirectData[ii].status != VEDIRECT_VALUE_STATE_UNDEFINED) {
        debugD("%s: %s (%lu ms)", this->VEDirectData[ii].label, this->VEDirectData[ii].valueConfirmed, millis() - this->VEDirectData[ii].lastupdate);
      }
    }
  });
  #endif
}

