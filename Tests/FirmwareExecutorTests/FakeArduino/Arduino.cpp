#include "Arduino.h"
#include "Wire.h"

HardwareSerial Serial;
HardwareSerial Serial2;
TwoWire Wire;
int testAnalogOutputs[32] = {};
int testDigitalOutputs[32] = {};
uint32_t testMillis = 0;
uint32_t testMicroseconds = 0;
int testPinModes[32] = {};
bool testSdaHeldLow=false, testSclHeldLow=false;
unsigned int testBusClockPulses=0, testSdaReleaseAfterPulses=0;
