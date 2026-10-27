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
  // raw retains the historical fixture angles at their calibrated reference.
  // Model the actual sensor's volatile zero register separately: power loss
  // changes the returned angle even when the mechanical shaft never moved.
  uint16_t zero[2] = {0, 2396};
  int zeroAfterAngle = -1;
  uint8_t zeroResponseBytes = 2;
  unsigned int transferUs = 0, beginCalls = 0, endCalls = 0, timeoutUs = 100000;
  bool recoveryAlwaysUnpowered = true;
  void begin() { ++beginCalls; }
  void end() { ++endCalls; recoveryAlwaysUnpowered = recoveryAlwaysUnpowered && !testAnalogOutputs[3] && !testAnalogOutputs[9]; }
  void setWireTimeout(unsigned int value) { timeoutUs=value; }
  void beginTransmission(uint8_t address) { axis = address == 0x41 ? 0 : 1; writes = 0; }
  void write(uint8_t value) { if(writes++ == 0) reg=value; }
  uint8_t endTransmission(bool = true) {
    delayMicroseconds(transferUs<timeoutUs ? transferUs : timeoutUs);
    if(transferUs>timeoutUs) return 5;
    return testSdaHeldLow || testSclHeldLow ? 2 : connected[axis] ? 0 : 4;
  }
  uint8_t requestFrom(uint8_t, uint8_t) {
    delayMicroseconds(transferUs<timeoutUs ? transferUs : timeoutUs);
    cursor = 0;
    word = reg==0x16 ? zero[axis] : uint16_t((raw[axis]+(axis==1 ? 2396 : 0)-zero[axis])&0x3FFF);
    count = reg==0x16 && zeroResponseBytes<responseBytes ? zeroResponseBytes : responseBytes;
    return transferUs>timeoutUs ? 0 : count;
  }
  int available() { return count - cursor; }
  int read() {
    int value=cursor++ == 0 ? word >> 6 : word & 0x3F;
    if(cursor==2 && reg==0xFE && axis==1 && zeroAfterAngle>=0) { zero[axis]=(uint16_t)zeroAfterAngle;zeroAfterAngle=-1; }
    return value;
  }
private:
  int axis = 0;
  int cursor = 0;
  unsigned int writes = 0;
  uint8_t reg = 0xFE, count = 0;
  uint16_t word = 0;
};
extern TwoWire Wire;
#endif
