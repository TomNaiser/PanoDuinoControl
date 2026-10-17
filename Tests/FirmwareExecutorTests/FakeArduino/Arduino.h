#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <deque>
#include <string>

#define strtok_r strtok_s
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define SDA 18
#define SCL 19
#define DEC 10
#define U_DEG 0

typedef unsigned char byte;
typedef bool boolean;

extern uint32_t testMillis;
inline unsigned long millis() { return testMillis; }
inline void delay(unsigned long value) { testMillis += (uint32_t)value; }
extern uint32_t testMicroseconds;
extern int testPinModes[32];
extern bool testSdaHeldLow, testSclHeldLow;
extern unsigned int testBusClockPulses, testSdaReleaseAfterPulses;
inline unsigned long micros() { return testMillis*1000UL+testMicroseconds; }
inline void delayMicroseconds(unsigned int value)
{ testMicroseconds+=value; testMillis+=testMicroseconds/1000; testMicroseconds%=1000; }
inline void pinMode(int pin, int mode)
{
  if(pin==SCL && testPinModes[pin]==OUTPUT && mode==INPUT_PULLUP)
  {
    ++testBusClockPulses;
    if(testSdaReleaseAfterPulses && testBusClockPulses>=testSdaReleaseAfterPulses) testSdaHeldLow=false;
  }
  testPinModes[pin]=mode;
}
extern int testDigitalOutputs[32];
inline void digitalWrite(int pin, int value) { testDigitalOutputs[pin] = value; }
inline int digitalRead(int pin)
{
  if((pin==SDA && testSdaHeldLow) || (pin==SCL && testSclHeldLow)) return LOW;
  return testPinModes[pin]==OUTPUT ? testDigitalOutputs[pin] : HIGH;
}
extern int testAnalogOutputs[32];
inline void analogWrite(int pin, int value) { testAnalogOutputs[pin] = value; }

class Stream
{
public:
  virtual ~Stream() {}
  virtual int available() = 0;
  virtual int peek() = 0;
  virtual int read() = 0;
  virtual std::size_t print(const char *value) = 0;
  virtual std::size_t write(unsigned char value) = 0;
  virtual int availableForWrite() { return 4096; }
};

class HardwareSerial : public Stream
{
public:
  void begin(unsigned long) {}
  operator bool() const { return true; }
  int available() { return static_cast<int>(input.size()); }
  int peek() { return input.empty() ? -1 : input.front(); }
  int read() { if(input.empty()) return -1; int value=input.front(); input.pop_front(); return value; }
  std::size_t print(const char *value) { output += value; return std::strlen(value); }
  std::size_t print(double, int = 2) { return 0; }
  std::size_t print(long, int = DEC) { return 0; }
  std::size_t print(unsigned long, int = DEC) { return 0; }
  std::size_t print(int, int = DEC) { return 0; }
  std::size_t println(const char *) { return 0; }
  std::size_t println(double, int = 2) { return 0; }
  std::size_t println(long, int = DEC) { return 0; }
  std::size_t println(unsigned long, int = DEC) { return 0; }
  std::size_t println(int, int = DEC) { return 0; }
  std::size_t println() { return 0; }
  std::size_t write(unsigned char value) { output += static_cast<char>(value); return 1; }
  void feed(const std::string &value) { for(unsigned char ch : value) input.push_back(ch); }
  std::deque<unsigned char> input;
  std::string output;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial2;

#endif
