#ifndef TEST_WIRE_H
#define TEST_WIRE_H
#include <cstdint>
#include "Arduino.h"
class TwoWire
{
public:
  bool connected[2] = {true, true};
  uint8_t responseBytes = 2;
  uint16_t raw[2] = {16383, 16383};
  unsigned int transferUs = 0, beginCalls = 0, endCalls = 0, timeoutUs = 100000;
  bool recoveryAlwaysUnpowered = true;
  void begin() { ++beginCalls; }
  void end() { ++endCalls; recoveryAlwaysUnpowered = recoveryAlwaysUnpowered && !testAnalogOutputs[3] && !testAnalogOutputs[9]; }
  void setWireTimeout(unsigned int value) { timeoutUs=value; }
  void beginTransmission(uint8_t address) { axis = address == 0x41 ? 0 : 1; }
  void write(uint8_t) {}
  uint8_t endTransmission(bool = true) {
    delayMicroseconds(transferUs<timeoutUs ? transferUs : timeoutUs);
    if(transferUs>timeoutUs) return 5;
    return testSdaHeldLow || testSclHeldLow ? 2 : connected[axis] ? 0 : 4;
  }
  uint8_t requestFrom(uint8_t, uint8_t) {
    delayMicroseconds(transferUs<timeoutUs ? transferUs : timeoutUs);
    cursor = 0; return transferUs>timeoutUs ? 0 : responseBytes;
  }
  int available() { return responseBytes - cursor; }
  int read() { return cursor++ == 0 ? raw[axis] >> 6 : raw[axis] & 0x3F; }
private:
  int axis = 0;
  int cursor = 0;
};
extern TwoWire Wire;
#endif
