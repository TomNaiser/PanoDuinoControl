#include "ProtocolV2.h"
#include <string.h>

#ifdef __AVR__
#define V2_FLASH(value) F(value)
#define V2_EQUALS(value, literal) (strcmp_P((value), PSTR(literal)) == 0)
#else
#define V2_FLASH(value) value
#define V2_EQUALS(value, literal) (strcmp((value), (literal)) == 0)
#endif

static const uint32_t FRAME_TIMEOUT_MS = 300;

ProtocolV2::ProtocolV2(Stream &stream, IV2FrameHandler &handler)
  : _stream(stream), _handler(handler), _receiveLength(0), _receiving(false),
    _frameStartedMs(0), _deviceSequence(1), _handlingSequence(0),
    _handlingRequestCrc(0), _handlingCacheableRequest(false),
    _hasCachedResult(false), _cachedSequence(0), _cachedRequestCrc(0),
    _cachedAccepted(false), _transmitCrc(0xFFFF), _transmitting(false)
{
  _receiveBuffer[0] = 0;
  _cachedResult[0] = 0;
}

void ProtocolV2::tick(uint32_t nowMs, bool preserveLegacyBytes)
{
  if (_receiving && (uint32_t)(nowMs - _frameStartedMs) > FRAME_TIMEOUT_MS)
  {
    resetReceiver();
  }

  unsigned int receiveBudget = 256;
  while (_stream.available() > 0 && receiveBudget--)
  {
    if (!_receiving)
    {
      if (_stream.peek() != '@')
      {
        if (preserveLegacyBytes) return; // Before V2 HELLO, leave V1 input to the legacy parser.
        // V1 commands are disabled during a V2 session. Consume UART noise or
        // a truncated frame tail so the next HELLO/ESTOP cannot remain hidden.
        _stream.read();
        continue;
      }
      _stream.read();
      startReceiver(nowMs);
      continue;
    }

    int raw = _stream.read();
    if (raw < 0) return;
    char value = (char)raw;
    if (value == '@')
    {
      startReceiver(nowMs);
      continue;
    }
    if (value == '\r') continue;
    if (value == '\n')
    {
      _receiveBuffer[_receiveLength] = 0;
      parseCompleteFrame(nowMs);
      resetReceiver();
      return; // At most one complete request per loop; give control a turn.
    }
    if (value < 32 || value > 126)
    {
      resetReceiver();
      continue;
    }
    if (_receiveLength >= V2_MAX_FRAME_LENGTH)
    {
      sendNack(0, V2_FLASH("FRAME_TOO_LONG"));
      resetReceiver();
      continue;
    }
    _receiveBuffer[_receiveLength++] = value;
  }
}

void ProtocolV2::startReceiver(uint32_t nowMs)
{
  _receiving = true;
  _receiveLength = 1;
  _receiveBuffer[0] = '@';
  _receiveBuffer[1] = 0;
  _frameStartedMs = nowMs;
}

void ProtocolV2::resetReceiver()
{
  _receiving = false;
  _receiveLength = 0;
  _receiveBuffer[0] = 0;
}

void ProtocolV2::parseCompleteFrame(uint32_t nowMs)
{
  if (_receiveLength < 10) return;
  char *crcSeparator = strrchr(_receiveBuffer, '|');
  if (crcSeparator == 0 || strlen(crcSeparator + 1) != 4)
  {
    sendNack(0, V2_FLASH("INVALID_CRC"));
    return;
  }

  uint16_t receivedCrc = 0;
  const char *crcText = crcSeparator + 1;
  for (uint8_t index = 0; index < 4; index++)
  {
    char digit = crcText[index];
    uint8_t value;
    if (digit >= '0' && digit <= '9') value = (uint8_t)(digit - '0');
    else if (digit >= 'A' && digit <= 'F') value = (uint8_t)(digit - 'A' + 10);
    else if (digit >= 'a' && digit <= 'f') value = (uint8_t)(digit - 'a' + 10);
    else
    {
      sendNack(0, V2_FLASH("INVALID_CRC"));
      return;
    }
    receivedCrc = (uint16_t)((receivedCrc << 4) | value);
  }
  size_t bodyLength = (size_t)(crcSeparator - _receiveBuffer);
  if (crc16(_receiveBuffer, bodyLength) != receivedCrc)
  {
    sendNack(0, V2_FLASH("CRC_ERROR"));
    return;
  }
  *crcSeparator = 0;

  char *savePointer = 0;
  char *prefix = strtok_r(_receiveBuffer, "|", &savePointer);
  char *sequenceText = strtok_r(0, "|", &savePointer);
  char *messageType = strtok_r(0, "|", &savePointer);
  if (prefix == 0 || !V2_EQUALS(prefix, "@P2"))
  {
    sendNack(0, V2_FLASH("UNSUPPORTED_VERSION"));
    return;
  }
  uint32_t sequenceValue;
  if (sequenceText == 0 || !parseUnsigned(sequenceText, 65535UL, sequenceValue) || messageType == 0)
  {
    sendNack(0, V2_FLASH("BAD_FIELD_COUNT"));
    return;
  }

  V2Frame frame;
  frame.sequence = (uint16_t)sequenceValue;
  frame.messageType = messageType;
  frame.fieldCount = 0;
  char *field;
  while ((field = strtok_r(0, "|", &savePointer)) != 0)
  {
    if (frame.fieldCount >= V2_MAX_FIELDS)
    {
      sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT"));
      return;
    }
    frame.fields[frame.fieldCount++] = field;
  }
  // Read-only messages may safely be repeated because they carry fresh state.
  // Mutating commands must instead return the exact previous result when the
  // client retries the same sequence after a lost ACK.
  bool cacheable = !V2_EQUALS(messageType, "HELLO") &&
                   !V2_EQUALS(messageType, "HEARTBEAT") &&
                   !V2_EQUALS(messageType, "GET_STATUS") &&
                   !V2_EQUALS(messageType, "GET_CAPABILITIES") &&
                   !V2_EQUALS(messageType, "GET_PLAN");
  if (cacheable && _hasCachedResult && frame.sequence == _cachedSequence)
  {
    if ((uint16_t)receivedCrc != _cachedRequestCrc)
    {
      sendNack(frame.sequence, V2_FLASH("SEQUENCE_CONFLICT"));
    }
    else if (_cachedAccepted)
    {
      sendAck(frame.sequence, _cachedResult);
    }
    else
    {
      sendNack(frame.sequence, _cachedResult);
    }
    return;
  }

  _handlingSequence = frame.sequence;
  _handlingRequestCrc = receivedCrc;
  _handlingCacheableRequest = cacheable;
  _handler.handleV2Frame(frame, *this, nowMs);
  _handlingCacheableRequest = false;
}

void ProtocolV2::sendAck(uint16_t requestSequence, const char *resultCode)
{
  const char *result = resultCode == 0 ? "OK" : resultCode;
  if (_handlingCacheableRequest && requestSequence == _handlingSequence)
  {
    _hasCachedResult = true;
    _cachedSequence = requestSequence;
    _cachedRequestCrc = _handlingRequestCrc;
    _cachedAccepted = true;
    strncpy(_cachedResult, result, sizeof(_cachedResult) - 1);
    _cachedResult[sizeof(_cachedResult) - 1] = 0;
  }
  beginFrame(V2_FLASH("ACK"));
  addField((uint32_t)requestSequence);
  addField(result);
  endFrame();
}

void ProtocolV2::sendNack(uint16_t requestSequence, const char *errorCode)
{
  const char *result = errorCode == 0 ? "FAULT" : errorCode;
  if (_handlingCacheableRequest && requestSequence == _handlingSequence)
  {
    _hasCachedResult = true;
    _cachedSequence = requestSequence;
    _cachedRequestCrc = _handlingRequestCrc;
    _cachedAccepted = false;
    strncpy(_cachedResult, result, sizeof(_cachedResult) - 1);
    _cachedResult[sizeof(_cachedResult) - 1] = 0;
  }
  beginFrame(V2_FLASH("NACK"));
  addField((uint32_t)requestSequence);
  addField(result);
  endFrame();
}

bool ProtocolV2::sendFrame(const char *messageType, const char *fields)
{
  beginFrame(messageType);
  if (fields != 0 && fields[0] != 0) addField(fields);
  return endFrame();
}

void ProtocolV2::beginFrame(const char *messageType)
{
  _transmitCrc = 0xFFFF;
  _transmitting = true;
  writeBodyText("@P2|");
  writeBodyUnsigned(_deviceSequence);
  writeBodyChar('|');
  writeBodyText(messageType);
}

#ifdef __AVR__
void ProtocolV2::beginFrame(const __FlashStringHelper *messageType)
{
  _transmitCrc = 0xFFFF;
  _transmitting = true;
  writeBodyFlash(F("@P2|"));
  writeBodyUnsigned(_deviceSequence);
  writeBodyChar('|');
  writeBodyFlash(messageType);
}
#endif

void ProtocolV2::addField(const char *value)
{
  if (!_transmitting) return;
  writeBodyChar('|');
  writeBodyText(value == 0 ? "" : value);
}

#ifdef __AVR__
void ProtocolV2::addField(const __FlashStringHelper *value)
{
  if (!_transmitting) return;
  writeBodyChar('|');
  writeBodyFlash(value);
}
#endif

void ProtocolV2::addField(uint32_t value)
{
  if (!_transmitting) return;
  writeBodyChar('|');
  writeBodyUnsigned(value);
}

void ProtocolV2::addSignedField(int32_t value)
{
  if (!_transmitting) return;
  writeBodyChar('|');
  if (value < 0)
  {
    writeBodyChar('-');
    // -(INT32_MIN) ist in int32_t nicht darstellbar.
    writeBodyUnsigned((uint32_t)(-(value + 1)) + 1UL);
  }
  else writeBodyUnsigned((uint32_t)value);
}

bool ProtocolV2::endFrame()
{
  if (!_transmitting) return false;
  const char hex[] = "0123456789ABCDEF";
  uint16_t crc = _transmitCrc;
  _stream.write('|');
  _stream.write(hex[(crc >> 12) & 0x0F]);
  _stream.write(hex[(crc >> 8) & 0x0F]);
  _stream.write(hex[(crc >> 4) & 0x0F]);
  _stream.write(hex[crc & 0x0F]);
  _stream.write('\r');
  _stream.write('\n');
  _transmitting = false;
  _deviceSequence++;
  if (_deviceSequence == 0) _deviceSequence = 1;
  return true;
}

void ProtocolV2::writeBodyChar(char value)
{
  _stream.write((uint8_t)value);
  _transmitCrc ^= (uint16_t)((uint8_t)value << 8);
  for (uint8_t bit = 0; bit < 8; bit++)
    _transmitCrc = (_transmitCrc & 0x8000) != 0
      ? (uint16_t)((_transmitCrc << 1) ^ 0x1021)
      : (uint16_t)(_transmitCrc << 1);
}

void ProtocolV2::writeBodyText(const char *value)
{
  if (value == 0) return;
  while (*value != 0) writeBodyChar(*value++);
}

#ifdef __AVR__
void ProtocolV2::writeBodyFlash(const __FlashStringHelper *value)
{
  PGM_P cursor = reinterpret_cast<PGM_P>(value);
  char current;
  while ((current = (char)pgm_read_byte(cursor++)) != 0) writeBodyChar(current);
}
#endif

void ProtocolV2::writeBodyUnsigned(uint32_t value)
{
  char digits[10];
  uint8_t count = 0;
  do
  {
    digits[count++] = (char)('0' + value % 10UL);
    value /= 10UL;
  } while (value != 0 && count < sizeof(digits));
  while (count > 0) writeBodyChar(digits[--count]);
}

uint16_t ProtocolV2::crc16(const char *data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++)
  {
    crc ^= (uint16_t)((uint8_t)data[i] << 8);
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x8000) != 0 ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

bool ProtocolV2::parseUnsigned(const char *value, uint32_t maximum, uint32_t &result)
{
  if (value == 0 || value[0] == 0 || value[0] == '-') return false;
  uint32_t parsed = 0;
  while (*value != 0)
  {
    if (*value < '0' || *value > '9') return false;
    uint8_t digit = (uint8_t)(*value++ - '0');
    if (parsed > (maximum - digit) / 10UL) return false;
    parsed = parsed * 10UL + digit;
  }
  result = parsed;
  return true;
}

#ifdef __AVR__
void ProtocolV2::sendAck(uint16_t requestSequence, const __FlashStringHelper *resultCode)
{
  PGM_P result = reinterpret_cast<PGM_P>(resultCode);
  if (_handlingCacheableRequest && requestSequence == _handlingSequence)
  {
    _hasCachedResult = true;
    _cachedSequence = requestSequence;
    _cachedRequestCrc = _handlingRequestCrc;
    _cachedAccepted = true;
    strncpy_P(_cachedResult, result, sizeof(_cachedResult) - 1);
    _cachedResult[sizeof(_cachedResult) - 1] = 0;
  }
  beginFrame(F("ACK"));
  addField((uint32_t)requestSequence);
  addField(resultCode);
  endFrame();
}

void ProtocolV2::sendNack(uint16_t requestSequence, const __FlashStringHelper *errorCode)
{
  PGM_P result = reinterpret_cast<PGM_P>(errorCode);
  if (_handlingCacheableRequest && requestSequence == _handlingSequence)
  {
    _hasCachedResult = true;
    _cachedSequence = requestSequence;
    _cachedRequestCrc = _handlingRequestCrc;
    _cachedAccepted = false;
    strncpy_P(_cachedResult, result, sizeof(_cachedResult) - 1);
    _cachedResult[sizeof(_cachedResult) - 1] = 0;
  }
  beginFrame(F("NACK"));
  addField((uint32_t)requestSequence);
  addField(errorCode);
  endFrame();
}
#endif
