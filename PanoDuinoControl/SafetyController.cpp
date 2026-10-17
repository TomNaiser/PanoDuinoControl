#include "SafetyController.h"
#if defined(ARDUINO_NANO_R4)
#include <EEPROM.h>
#endif

struct StoredTiltLimits
{
  uint32_t magic;
  int32_t minimum;
  int32_t maximum;
  uint32_t check;
};

static const uint32_t TILT_STORE_MAGIC = 0x54494C54UL;
struct StoredTiltZero { uint32_t magic; int32_t offset; uint32_t check; };
static const uint32_t TILT_ZERO_MAGIC = 0x5A45524FUL;
static uint32_t tiltZeroCheck(int32_t offset) { return TILT_ZERO_MAGIC ^ (uint32_t)offset ^ 0x7352A9C1UL; }
static uint32_t tiltStoreCheck(int32_t minimum, int32_t maximum)
{
  return TILT_STORE_MAGIC ^ (uint32_t)minimum ^ ((uint32_t)maximum << 1) ^ 0xA5C37E19UL;
}

#if defined(PANODUINO_HOST_TEST)
static StoredTiltLimits hostStoredTiltLimits = { 0, 0, 0, 0 };
static StoredTiltZero hostStoredTiltZero = { 0, 0, 0 };
#endif

SafetyController::SafetyController()
  : _motionEnabled(false), _lastFault("NONE"),
    _tiltMinimumMdeg(TILT_HARD_MIN_MDEG), _tiltMaximumMdeg(TILT_HARD_MAX_MDEG), _tiltZeroOffsetMdeg(0), _bootId(0)
{
}

void SafetyController::beginBootSession()
{
#if defined(ARDUINO_NANO_R4)
  // Two independent slots preserve the previous identity if power fails during a write.
  // Tilt limits occupy bytes 0..15; boot slots start at 32 and 48.
  struct BootRecord { uint32_t magic, id, check; };
  const uint32_t magic = 0x424F4F54UL;
  BootRecord a, b;
  EEPROM.get(32, a); EEPROM.get(48, b);
  bool validA = a.magic == magic && a.id != 0 && a.check == (magic ^ a.id ^ 0xAC53F192UL);
  bool validB = b.magic == magic && b.id != 0 && b.check == (magic ^ b.id ^ 0xAC53F192UL);
  bool latestA = validA && (!validB || (int32_t)(a.id - b.id) > 0);
  uint32_t previous = latestA ? a.id : validB ? b.id : 0;
  uint32_t next = previous + 1; if (next == 0) next = 1;
  BootRecord record = { magic, next, magic ^ next ^ 0xAC53F192UL }, verified;
  int address = latestA ? 48 : 32;
  EEPROM.put(address, record); EEPROM.get(address, verified);
  _bootId = verified.magic == magic && verified.id == next && verified.check == record.check ? next : 0;
#elif defined(PANODUINO_HOST_TEST)
  static uint32_t hostBootId = 0;
  _bootId = ++hostBootId;
#else
  _bootId = 0; // Older boards provide no persistent session identity.
#endif
}

void SafetyController::loadTiltLimits()
{
  StoredTiltLimits stored = { 0, 0, 0, 0 };
#if defined(ARDUINO_NANO_R4)
  EEPROM.get(0, stored);
#elif defined(PANODUINO_HOST_TEST)
  stored = hostStoredTiltLimits;
#endif
  if (stored.magic == TILT_STORE_MAGIC &&
      stored.check == tiltStoreCheck(stored.minimum, stored.maximum) &&
      stored.minimum >= TILT_ABSOLUTE_MIN_MDEG && stored.maximum <= TILT_ABSOLUTE_MAX_MDEG &&
      stored.minimum < stored.maximum)
  {
    _tiltMinimumMdeg = stored.minimum;
    _tiltMaximumMdeg = stored.maximum;
  }
  StoredTiltZero zero = { 0, 0, 0 };
#if defined(ARDUINO_NANO_R4)
  EEPROM.get(16, zero);
#elif defined(PANODUINO_HOST_TEST)
  zero = hostStoredTiltZero;
#endif
  if (zero.magic == TILT_ZERO_MAGIC && zero.check == tiltZeroCheck(zero.offset) &&
      zero.offset >= TILT_ABSOLUTE_MIN_MDEG && zero.offset <= TILT_ABSOLUTE_MAX_MDEG)
    _tiltZeroOffsetMdeg = zero.offset;
}

bool SafetyController::setTiltZeroOffsetMdeg(int32_t offsetMdeg)
{
  if (offsetMdeg < TILT_ABSOLUTE_MIN_MDEG || offsetMdeg > TILT_ABSOLUTE_MAX_MDEG) return false;
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
  StoredTiltZero stored = { TILT_ZERO_MAGIC, offsetMdeg, tiltZeroCheck(offsetMdeg) }, verified;
#if defined(ARDUINO_NANO_R4)
  EEPROM.put(16, stored); EEPROM.get(16, verified);
#else
  hostStoredTiltZero = stored; verified = hostStoredTiltZero;
#endif
  if (verified.magic != stored.magic || verified.offset != stored.offset || verified.check != stored.check) return false;
  _tiltZeroOffsetMdeg = offsetMdeg; return true;
#else
  return false;
#endif
}

bool SafetyController::setTiltLimits(int32_t minimumMdeg, int32_t maximumMdeg)
{
  int64_t rawMinimum = (int64_t)minimumMdeg + _tiltZeroOffsetMdeg;
  int64_t rawMaximum = (int64_t)maximumMdeg + _tiltZeroOffsetMdeg;
  if (rawMinimum < TILT_ABSOLUTE_MIN_MDEG || rawMaximum > TILT_ABSOLUTE_MAX_MDEG || minimumMdeg >= maximumMdeg) return false;
  minimumMdeg = (int32_t)rawMinimum; maximumMdeg = (int32_t)rawMaximum;
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
  StoredTiltLimits stored = { TILT_STORE_MAGIC, minimumMdeg, maximumMdeg,
                              tiltStoreCheck(minimumMdeg, maximumMdeg) };
#if defined(ARDUINO_NANO_R4)
  EEPROM.put(0, stored);
  StoredTiltLimits verified;
  EEPROM.get(0, verified);
#else
  hostStoredTiltLimits = stored;
  StoredTiltLimits verified = hostStoredTiltLimits;
#endif
  if (verified.magic != stored.magic || verified.minimum != stored.minimum ||
      verified.maximum != stored.maximum || verified.check != stored.check) return false;
  _tiltMinimumMdeg = minimumMdeg;
  _tiltMaximumMdeg = maximumMdeg;
  return true;
#else
  return false;
#endif
}

SafetyResult SafetyController::validateTarget(int32_t panMdeg, int32_t tiltMdeg) const
{
  if (!limitsConfirmed()) return SAFETY_LIMITS_NOT_CONFIRMED;
  if (panMdeg < PAN_HARD_MIN_MDEG || panMdeg > PAN_HARD_MAX_MDEG) return SAFETY_PAN_LIMIT;
  if (tiltMdeg < tiltMinimumMdeg() || tiltMdeg > tiltMaximumMdeg()) return SAFETY_TILT_LIMIT;
  return SAFETY_OK;
}

SafetyResult SafetyController::observeTilt(int32_t tiltMdeg)
{
  if (!limitsConfirmed())
  {
    stopMotion();
    _lastFault = "SAFETY_UNCONFIRMED";
    return SAFETY_LIMITS_NOT_CONFIRMED;
  }
  if (tiltMdeg < tiltMinimumMdeg() || tiltMdeg > tiltMaximumMdeg())
  {
    stopMotion();
    _lastFault = "FAULT_TILT_LIMIT";
    return SAFETY_TILT_LIMIT;
  }
  return SAFETY_OK;
}

static bool manualAxisTargetAllowed(int32_t actual, int32_t target, int32_t minimum, int32_t maximum)
{
  if (actual < minimum) return target >= actual && target <= maximum;
  if (actual > maximum) return target <= actual && target >= minimum;
  return target >= minimum && target <= maximum;
}

SafetyResult SafetyController::validateManualTarget(int32_t actualPan, int32_t actualTilt, int32_t targetPan, int32_t targetTilt) const
{
  if (!limitsConfirmed()) return SAFETY_LIMITS_NOT_CONFIRMED;
  if (!manualAxisTargetAllowed(actualPan, targetPan, PAN_HARD_MIN_MDEG, PAN_HARD_MAX_MDEG)) return SAFETY_PAN_LIMIT;
  if (!manualAxisTargetAllowed(actualTilt, targetTilt, tiltMinimumMdeg(), tiltMaximumMdeg())) return SAFETY_TILT_LIMIT;
  // While outside, hold each unaffected axis and make measurable inward progress.
  bool panOutside = actualPan < PAN_HARD_MIN_MDEG || actualPan > PAN_HARD_MAX_MDEG;
  bool tiltOutside = actualTilt < tiltMinimumMdeg() || actualTilt > tiltMaximumMdeg();
  if (panOutside || tiltOutside)
  {
    if (!panOutside && actualPan != targetPan) return SAFETY_PAN_LIMIT;
    if (!tiltOutside && actualTilt != targetTilt) return SAFETY_TILT_LIMIT;
    if (actualPan == targetPan && actualTilt == targetTilt) return SAFETY_TILT_LIMIT;
  }
  return SAFETY_OK;
}

int SafetyController::limitDrive(bool tiltAxis, int32_t positionMdeg, int signedPwm) const
{
  if (!motionEnabled() || signedPwm == 0) return 0;
  int32_t minimum = tiltAxis ? tiltMinimumMdeg() : PAN_HARD_MIN_MDEG;
  int32_t maximum = tiltAxis ? tiltMaximumMdeg() : PAN_HARD_MAX_MDEG;
  int64_t distance = signedPwm < 0 ? (int64_t)positionMdeg - minimum : (int64_t)maximum - positionMdeg;
  if (distance <= 0) return 0; // Inward polarity remains possible at and beyond a limit.
  return signedPwm;
}

bool SafetyController::limitsConfirmed() const
{
  return SAFETY_LIMITS_CONFIRMED == 1;
}

bool SafetyController::motionEnabled() const
{
  return _motionEnabled && limitsConfirmed();
}

void SafetyController::enableMotion()
{
  _motionEnabled = limitsConfirmed();
  if (!_motionEnabled) _lastFault = "SAFETY_UNCONFIRMED";
}

void SafetyController::stopMotion()
{
  _motionEnabled = false;
}

const char *SafetyController::lastFault() const
{
  return _lastFault;
}
