#ifndef PANODUINO_SERIAL_TRANSPORT_H
#define PANODUINO_SERIAL_TRANSPORT_H
#include <Arduino.h>

// Queue protocol output so whole frames never wait on the UART.
// Renesas core 1.6.0 UART::write is synchronous and has no availableForWrite
// override. Its explicit mode sends at most 8 bytes (~1.4 ms at 57600 baud).
// Buffered implementations drain only available space.
class SerialTransport : public Stream
{
public:
  explicit SerialTransport(Stream &serial, bool synchronousUart = false)
    : _serial(serial), _head(0), _count(0), _overflow(false), _synchronousUart(synchronousUart) {}
  int available() { return _serial.available(); }
  int peek() { return _serial.peek(); }
  int read() { return _serial.read(); }
  int availableForWrite() { return (int)(CAPACITY - _count); }
  size_t write(uint8_t value)
  {
    if (_count == CAPACITY) { _overflow = true; return 0; }
    _queue[(_head + _count) % CAPACITY] = value; ++_count; return 1;
  }
  void flush() { service(); } // Never a blocking UART flush.
#ifdef PANODUINO_HOST_TEST
  size_t print(const char *value)
  { size_t count = 0; while (*value) count += write((uint8_t)*value++); return count; }
#else
  using Print::write;
#endif
  void service()
  {
    unsigned int budget = _synchronousUart ? 8 : 128;
    while (_count && budget-- && (_synchronousUart || _serial.availableForWrite() > 0))
    {
      if (_serial.write(_queue[_head]) != 1) break;
      _head = (uint16_t)((_head + 1) % CAPACITY); --_count;
    }
  }
  bool takeOverflow() { bool overflow = _overflow; _overflow = false; return overflow; }
  // A disconnected USB bench owner cannot receive queued status frames.
  void discardPending() { _head = _count = 0; _overflow = false; }
private:
#ifdef __AVR__
  static const uint16_t CAPACITY = 128; // Advanced motion acceptance targets the R4.
#else
  static const uint16_t CAPACITY = 2048;
#endif
  Stream &_serial;
  uint8_t _queue[CAPACITY];
  uint16_t _head, _count;
  bool _overflow, _synchronousUart;
};
#endif
