#ifndef PANODUINO_DEBUG_EVENT_LOG_H
#define PANODUINO_DEBUG_EVENT_LOG_H

#include <stdint.h>

// Auf dem Nano R4 ueberbrueckt ein groesserer Ring Verbindungsunterbrechungen.
// Der verbundene Client protokolliert weiterhin den vollstaendigen Strom.
// Aus Istposition und quantisiertem
// Fehler kann der Supportclient die ungefaehre damalige Zielposition ableiten.
#if defined(ARDUINO_NANO_R4)
const uint8_t DEBUG_EVENT_CAPACITY_V2 = 64;
#else
const uint8_t DEBUG_EVENT_CAPACITY_V2 = 3;
#endif

enum DebugEventCodeV2
{
  DEBUG_BOOT = 0,
  DEBUG_SESSION_OPENED,
  DEBUG_PLAN_LOADED,
  DEBUG_RUN_STARTED,
  DEBUG_TARGET_SELECTED,
  DEBUG_TARGET_REACHED,
  DEBUG_SETTLING_LOST,
  DEBUG_TRIGGER_STARTED,
  DEBUG_IMAGE_COMMITTED,
  DEBUG_PAUSE_PENDING,
  DEBUG_PAUSED,
  DEBUG_RESUMED,
  DEBUG_REWOUND,
  DEBUG_MANUAL_MOVE_STARTED,
  DEBUG_MANUAL_MOVE_REACHED,
  DEBUG_MANUAL_TRIGGERED,
  DEBUG_STOPPED,
  DEBUG_COMPLETED,
  DEBUG_FAULT,
  DEBUG_FORWARDED,
  DEBUG_STEP_MOVE_TIMEOUT,
  DEBUG_STEP_TARGET_MISSED
};

struct DebugEventRecordV2
{
  uint16_t sequence;
  uint32_t deviceTimeMs;
  uint16_t imageIndex;
  int16_t actualPanDeciDeg;
  int16_t actualTiltDeciDeg;
  int16_t panErrorDeciDeg;
  int16_t tiltErrorDeciDeg;
  uint8_t code;
  uint8_t state;
  uint8_t phase;
  uint8_t stableSamples;
};

class DebugEventLogV2
{
public:
  DebugEventLogV2() : _start(0), _count(0), _nextSequence(1), _dropped(0) {}

  void clear()
  {
    _start = 0;
    _count = 0;
    _dropped = 0;
  }

  void append(const DebugEventRecordV2 &source)
  {
    uint8_t index;
    if (_count < DEBUG_EVENT_CAPACITY_V2)
    {
      index = (uint8_t)((_start + _count) % DEBUG_EVENT_CAPACITY_V2);
      _count++;
    }
    else
    {
      index = _start;
      _start = (uint8_t)((_start + 1) % DEBUG_EVENT_CAPACITY_V2);
      _dropped++;
    }
    _records[index] = source;
    _records[index].sequence = _nextSequence++;
    if (_nextSequence == 0) _nextSequence = 1;
  }

  uint8_t count() const { return _count; }
  uint16_t dropped() const { return _dropped; }
  uint16_t newestSequence() const
  {
    return _count == 0 ? 0 : record(_count - 1).sequence;
  }
  const DebugEventRecordV2 &record(uint8_t offset) const
  {
    return _records[(_start + offset) % DEBUG_EVENT_CAPACITY_V2];
  }

private:
  DebugEventRecordV2 _records[DEBUG_EVENT_CAPACITY_V2];
  uint8_t _start;
  uint8_t _count;
  uint16_t _nextSequence;
  uint16_t _dropped;
};

#endif
