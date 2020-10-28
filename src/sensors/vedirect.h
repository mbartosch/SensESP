#ifndef _vedirect_H_
#define _vedirect_H_

#include "sensor.h"
#include <system/observablevalue.h>

// SignalK sensor class for passively reading data from a Victron
// BMV600, BMV700, MPPT or Phoenix device.
// 
// 2020-10-26 Martin Bartosch for SY Entropy



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
  // by (factor/1000). Corresponding VEDirectData record must be float.
  // set to 1000 for a factor of 1
  // set to 1 for a factor of 1/1000 (e. g. to convert mV to V)
  // Special values:
  // set to 0 in order to keep the literal string (corresponding VEDirectData record must be String)
  // set to -1 in order to convert to a boolean 0/1 value (corresponding VEDirectData record must be int)
  // set to -2 in order to convert to an integer value (corresponding VEDirectData record must be int)
  // set to -273 for a conversion from °C to K (corresponding VEDirectData record must be float)
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

  union {
    void                    *genericValue; // only used for storing the pointer to the underlying struct member in this->data
    ObservableValue<float>  *floatValue;   // used to access a float value (unit_transform_factor >= 0)
    ObservableValue<int>    *intValue;     // used to access a float value (unit_transform_factor == -2)
    ObservableValue<String> *stringValue;  // used to access a float value (unit_transform_factor == 0)
  };
};


struct VEDirectData {
  ObservableValue<float>  mainVoltage;
  ObservableValue<float>  auxVoltage;
  ObservableValue<float>  midPointVoltage;
  ObservableValue<float>  midPointDeviation;
  ObservableValue<float>  panelVoltage;
  ObservableValue<float>  panelPower;
  ObservableValue<float>  batteryCurrent;
  ObservableValue<float>  loadCurrent;
  ObservableValue<int>    loadOutputState;
  ObservableValue<float>  batteryTemperature;
  ObservableValue<float>  instantaneousPower;
  ObservableValue<float>  consumedCoulombs;
  ObservableValue<float>  stateOfCharge;
  ObservableValue<float>  timeToGo;
  ObservableValue<int>    alarmState;
  ObservableValue<int>    relayState;
  ObservableValue<String> alarmReason;
  ObservableValue<float>  depthOfDeepestDischarge;
  ObservableValue<float>  depthOfLastDischarge;
  ObservableValue<float>  depthOfAverageDischarge;
  ObservableValue<int>    totalChargeCycles;
  ObservableValue<int>    totalFullCycles;
  ObservableValue<float>  cumulativeCoulombsDrawn;
  ObservableValue<float>  minimumMainVoltage;
  ObservableValue<float>  maximumMainVoltage;
  ObservableValue<float>  timeSinceLastFullCharge;
  ObservableValue<int>    totalAutomaticSynchronizations;
  ObservableValue<int>    totalLowMainVoltageAlarms;
  ObservableValue<int>    totalHighMainVoltageAlarms;
  ObservableValue<int>    totalLowAuxVoltageAlarms;
  ObservableValue<int>    totalHighAuxVoltageAlarms;
  ObservableValue<float>  minimumAuxVoltage;
  ObservableValue<float>  maximumAuxVoltage;
  ObservableValue<float>  dischargedEnergy;
  ObservableValue<float>  chargedEnergy;
  ObservableValue<float>  totalYield;
  ObservableValue<float>  todaysYield;
  ObservableValue<float>  todaysMaximumPower;
  ObservableValue<float>  yesterdaysYield;
  ObservableValue<float>  yesterdaysMaximumPower;
  ObservableValue<String> errorCode;
  ObservableValue<String> stateOfOperation;
  ObservableValue<String> modelDescription;
  ObservableValue<String> firmwareVersion;
  ObservableValue<String> productID;
  ObservableValue<String> serialNumber;
  ObservableValue<int>    daySequenceNumber;
  ObservableValue<String> deviceMode;
  ObservableValue<float>  acOutputVoltage;
  ObservableValue<float>  acOutputCurrent;
  ObservableValue<String> warningReason;
};


class VEDirectInput : public Sensor, public ValueProducer<String> {
 public:
  VEDirectInput(Stream* rx_stream, String config_path="", byte signalKUnitTransform = 1);
  virtual void enable() override final;

  // determine the index of the specified label
  // returns the index of the specified label or -1 if not found
  byte getVEDirectIndex(const char * label);

  // class operation mode (default: 1)
  // If set to 1, a SignalK compatible unit transform is
  // applied, resulting in a (possibly fractional) SI value
  // in valueConfirmed with unit meta_unit.
  // If set to 0, committed values to field valueConfirmed entry
  // are literally taken from the device. In this mode the class
  // does NOT update the data member variable and as a consequence
  // it is not possible to submit this data to a SignalK server.
  byte mode_signalKUnitTransform;

  // set SignalK value transform mode
  void setSignalKUnitTransform(byte mode);

  VEDirectData data;

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
  VEDirectFieldEntry VEDirectFields[VEDIRECT_TOTAL_LABELS];

  void clearLabelBuffer();
  void clearValueBuffer();
  void processField();
  void commit();
};


// wiring helpers
VEDirectInput* setup_vedirect_bmv(Stream* rx_stream, std::string deviceName);
//VEDirectInput* setup_vedirect_bmv600(Stream* rx_stream, std::string deviceName);
//VEDirectInput* setup_vedirect_bmv700(Stream* rx_stream, std::string deviceName);
//VEDirectInput* setup_vedirect_mppt(Stream* rx_stream, std::string deviceName);
//VEDirectInput* setup_vedirect_phoenix(Stream* rx_stream, std::string deviceName);



#endif
