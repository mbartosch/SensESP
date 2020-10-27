#ifndef _vedirect_H_
#define _vedirect_H_

#include "sensor.h"

// SignalK sensor class for passively reading data from a Victron
// BMV600, BMV700, MPPT or Phoenix device.
// 
// 2020-10-26 Martin Bartosch for SY Entropy


// Support for a Victron VE.Direct device over a serial interface
// receive state machine states
const byte VEDIRECT_RX_STATE_UNKNOWN = 0;
const byte VEDIRECT_RX_STATE_LABEL = 1;
const byte VEDIRECT_RX_STATE_VALUE = 2;
const byte VEDIRECT_RX_STATE_RECORDCOMPLETE = 3;

// max. buffer sizes according to VE.Direct specification
const byte VEDIRECT_BUF_LABEL_SIZE = 9;
const byte VEDIRECT_BUF_VALUE_SIZE = 33;

// max. number of VE.Direct labels according to Victron
const byte VEDIRECT_TOTAL_LABELS = 51;

// state definitions
const byte VEDIRECT_VALUE_STATE_UNDEFINED      = -1;
const byte VEDIRECT_VALUE_STATE_DATA_DIRTY     = 0;
const byte VEDIRECT_VALUE_STATE_DATA_VALID     = 1;

// one single field
struct VEDirectFieldEntry {
  // field label
  char label[VEDIRECT_BUF_LABEL_SIZE];
  // SignalK Unit transform factor, raw measurement will be multiplied
  // by (factor/1000).
  // set to 1000 for a factor of 1
  // set to 1 for a factor of 1/1000 (e. g. to convert mV to V)
  // Special values:
  // set to 0 in order to keep the literal string
  // set to -1 in order to convert to a boolean 0/1 value
  // set to -273 for a conversion from °C to K
  int unit_transform_factor;
  // SignalK requires measturement data in SI units. meta_unit
  // contains the unit to be supplied to SignalK.
  char meta_unit[6];

  // raw value read from the source
  char valuePending[VEDIRECT_BUF_VALUE_SIZE];
  // confirmed/committed value; if class operation
  // mode signalKUnitTransform is 1 the value
  // read from valuePending is converted to the SignalK
  // compatible "meta_unit" value during commit.
  char valueConfirmed[VEDIRECT_BUF_VALUE_SIZE];
  // timestamp of last successful update
  uint32_t lastupdate;
  byte status;
};

class VEDirectInput : public Sensor {
 public:
  VEDirectInput(Stream* rx_stream, String config_path="", byte signalKUnitTransform = 1);
  virtual void enable() override final;

  // determine the index of the specified label
  // returns the index of the specified label or -1 if not found
  byte getVEDirectIndex(const char * label);

  // class operation mode (default: 1)
  // if set to 0, committed values to field valueConfirmed entry
  // are literally taken from the device
  // if set to 1, a SignalK compatible unit transform is
  // applied, resulting in a (possibly fractional) SI value
  // in valueConfirmed with unit meta_unit.
  byte mode_signalKUnitTransform;

  // set SignalK value transform mode
  void setSignalKUnitTransform(byte mode);

 private:
   // Input data stream from VE.Direct serial port
  Stream* rx_stream;

  // receive buffers
  char fieldLabel[VEDIRECT_BUF_LABEL_SIZE];   // receive buffer for VE.Direct field
  char fieldValue[VEDIRECT_BUF_VALUE_SIZE];  // receive buffer for VE.Direct value
  byte rxBufIndex;      // index into current buffer
  byte checksum;        // checksum of received data
  byte receiveState;    // receive state machine


  // data field storage
  VEDirectFieldEntry VEDirectData[VEDIRECT_TOTAL_LABELS];

  void clearLabelBuffer();
  void clearValueBuffer();
  void processField();
  void commit();
};

#endif
