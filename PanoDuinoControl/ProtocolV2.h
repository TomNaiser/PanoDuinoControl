#ifndef PROTOCOL_V2_H
#define PROTOCOL_V2_H

#include <Arduino.h>
#include <stdint.h>

// Laengster gueltiger eingehender Rahmen ist LOAD_PLAN mit 16 Feldern
// (maximal 163 Zeichen bei voll ausgenutzten Integerwerten). 176 Zeichen
// lassen Reserve, ohne auf dem Nano 257 Byte dauerhaft zu blockieren.
#ifdef __AVR__
const uint16_t V2_MAX_FRAME_LENGTH = 164;
#else
const uint16_t V2_MAX_FRAME_LENGTH = 220;
#endif
const uint8_t V2_MAX_FIELDS = 20;

struct V2Frame
{
  uint16_t sequence;
  const char *messageType;
  const char *fields[V2_MAX_FIELDS];
  uint8_t fieldCount;
};

class ProtocolV2;

class IV2FrameHandler
{
public:
  virtual ~IV2FrameHandler() {}
  virtual void handleV2Frame(const V2Frame &frame, ProtocolV2 &protocol, uint32_t nowMs) = 0;
};

class ProtocolV2
{
public:
  ProtocolV2(Stream &stream, IV2FrameHandler &handler);
  void tick(uint32_t nowMs, bool preserveLegacyBytes = true);
  bool isReceiving() const { return _receiving; }
  int transmitSpace() { return _stream.availableForWrite(); }

  void sendAck(uint16_t requestSequence, const char *resultCode);
  void sendNack(uint16_t requestSequence, const char *errorCode);
  bool sendFrame(const char *messageType, const char *fields);
  void beginFrame(const char *messageType);
  void addField(const char *value);
  void addField(uint32_t value);
  void addSignedField(int32_t value);
  bool endFrame();
#ifdef __AVR__
  void sendAck(uint16_t requestSequence, const __FlashStringHelper *resultCode);
  void sendNack(uint16_t requestSequence, const __FlashStringHelper *errorCode);
  void beginFrame(const __FlashStringHelper *messageType);
  void addField(const __FlashStringHelper *value);
#endif

  static uint16_t crc16(const char *data, size_t length);

private:
  void resetReceiver();
  void startReceiver(uint32_t nowMs);
  void parseCompleteFrame(uint32_t nowMs);
  static bool parseUnsigned(const char *value, uint32_t maximum, uint32_t &result);
  void writeBodyChar(char value);
  void writeBodyText(const char *value);
#ifdef __AVR__
  void writeBodyFlash(const __FlashStringHelper *value);
#endif
  void writeBodyUnsigned(uint32_t value);

  Stream &_stream;
  IV2FrameHandler &_handler;
  char _receiveBuffer[V2_MAX_FRAME_LENGTH + 1];
  uint16_t _receiveLength;
  bool _receiving;
  uint32_t _frameStartedMs;
  uint16_t _deviceSequence;
  uint16_t _handlingSequence;
  uint16_t _handlingRequestCrc;
  bool _handlingCacheableRequest;
  bool _hasCachedResult;
  uint16_t _cachedSequence;
  uint16_t _cachedRequestCrc;
  bool _cachedAccepted;
  // Laengster cachebarer Ergebniscode ist MANUAL_ACTION_ACTIVE (20 Zeichen).
  char _cachedResult[21];
  uint16_t _transmitCrc;
  bool _transmitting;
};

#endif
