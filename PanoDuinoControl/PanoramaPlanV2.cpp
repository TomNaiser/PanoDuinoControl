#include "PanoramaPlanV2.h"
#include <limits.h>

PanoramaPlanV2::PanoramaPlanV2()
  : runId(0), planVersion(1), moveMode(0), startPanMdeg(0), startTiltMdeg(0),
    columns(0), rows(0), stepPanMdeg(0), stepTiltMdeg(0), endPanMdeg(0), endTiltMdeg(0), settleTimeMs(0),
    triggerPulseMs(30), exposureWaitMs(0), panSpeedPermille(1000),
    tiltSpeedPermille(1000), disconnectPolicy(DISCONNECT_CONTINUE), heartbeatTimeoutMs(0)
{
}

uint32_t PanoramaPlanV2::totalImages() const
{
  if (moveMode == 1)
  {
    uint64_t total = 0;
    for (uint16_t band = 0; band < rows; band++)
    {
      uint32_t count = shotsInBand(band);
      if (count == 0) return 0;
      total += count;
      if (total > UINT32_MAX) return UINT32_MAX;
    }
    return (uint32_t)total;
  }
  return (uint32_t)columns * (uint32_t)rows;
}

static int64_t roundDivide(int64_t value, int64_t divisor)
{
  return value >= 0 ? (value + divisor / 2) / divisor : (value - divisor / 2) / divisor;
}

uint32_t PanoramaPlanV2::shotsInBand(uint16_t band) const
{
  if (moveMode != 1 || band >= rows || rows == 0 || stepPanMdeg <= 0) return 0;
  int64_t tilt = rows == 1 ? startTiltMdeg
    : (int64_t)startTiltMdeg + roundDivide(((int64_t)endTiltMdeg - startTiltMdeg) * band, rows - 1);
  if (tilt < -90000 || tilt > 90000) return 0;
  int64_t span = (int64_t)endPanMdeg - startPanMdeg;
  if (span < 0) span = -span;
  if (span == 0) return 1;
  int64_t radians = (tilt < 0 ? -tilt : tilt) * 3141593LL / 180000LL;
  int64_t square = radians * radians / 1000000LL;
  int64_t fourth = square * square / 1000000LL;
  int64_t sixth = fourth * square / 1000000LL;
  int64_t cosine = 1000000LL - square / 2 + fourth / 24 - sixth / 720;
  if (cosine <= 0) return 0;
  int64_t denominator = (int64_t)stepPanMdeg * 1000000LL;
  uint64_t intervals = (uint64_t)((span * cosine + denominator - 1) / denominator);
  return (uint32_t)(span == 360000 ? (intervals == 0 ? 1 : intervals) : intervals + 1);
}

bool PanoramaPlanV2::targetFor(uint32_t imageIndex, TargetPositionV2 &target) const
{
  if (rows == 0 || imageIndex >= totalImages()) return false;
  if (moveMode == 1)
  {
    uint32_t remaining = imageIndex;
    for (uint16_t band = 0; band < rows; band++)
    {
      uint32_t shots = shotsInBand(band);
      if (remaining >= shots) { remaining -= shots; continue; }
      int64_t panSpan = (int64_t)endPanMdeg - startPanMdeg;
      uint32_t intervals = ((panSpan == 360000 || panSpan == -360000) ? shots : shots - 1);
      int64_t pan = intervals == 0 ? startPanMdeg
        : (int64_t)startPanMdeg + roundDivide(panSpan * remaining, intervals);
      int64_t tilt = rows == 1 ? startTiltMdeg
        : (int64_t)startTiltMdeg + roundDivide(((int64_t)endTiltMdeg - startTiltMdeg) * band, rows - 1);
      if (pan < INT32_MIN || pan > INT32_MAX || tilt < INT32_MIN || tilt > INT32_MAX || remaining > UINT16_MAX) return false;
      target.imageIndex = imageIndex;
      target.column = (uint16_t)remaining;
      target.row = band;
      target.panMdeg = (int32_t)pan;
      target.tiltMdeg = (int32_t)tilt;
      return true;
    }
    return false;
  }
  uint32_t column = imageIndex / rows;
  uint32_t row = imageIndex % rows;
  int64_t pan = (int64_t)startPanMdeg + (int64_t)column * stepPanMdeg;
  int64_t tilt = (int64_t)startTiltMdeg + (int64_t)row * stepTiltMdeg;
  if (pan < INT32_MIN || pan > INT32_MAX || tilt < INT32_MIN || tilt > INT32_MAX) return false;
  target.imageIndex = imageIndex;
  target.column = (uint16_t)column;
  target.row = (uint16_t)row;
  target.panMdeg = (int32_t)pan;
  target.tiltMdeg = (int32_t)tilt;
  return true;
}

bool PanoramaPlanV2::timingValid(uint32_t settleMs, uint32_t triggerMs, uint32_t exposureMs)
{
  return settleMs <= 600000UL && triggerMs > 0 && triggerMs <= 60000UL && exposureMs <= 3600000UL;
}

const char *PanoramaPlanV2::validate(const SafetyController &safety) const
{
  if (!safety.limitsConfirmed()) return "SAFETY_UNCONFIRMED";
  if (runId == 0) return "VALUE_OUT_OF_RANGE";
  if (moveMode > 1) return "UNSUPPORTED_MODE";
  if (planVersion != (moveMode == 1 ? 2 : 1)) return "UNSUPPORTED_VERSION";
  if (columns == 0 || rows == 0) return "VALUE_OUT_OF_RANGE";
  // Die Nano-Variante fuehrt bis zu 4000 Bilder autonom aus. Dadurch bleiben
  // Bild- und Segmentjournale kompakt, waehrend der Client alle Ereignisse
  // dauerhaft in CaptureManifest und Supportlog schreibt.
  if (totalImages() > 4000UL) return "VALUE_OUT_OF_RANGE";
  if (moveMode == 1)
  {
    if (rows == 1 && startTiltMdeg != endTiltMdeg) return "VALUE_OUT_OF_RANGE";
    if (stepPanMdeg <= 0 || totalImages() == 0) return "VALUE_OUT_OF_RANGE";
    uint32_t maximum = 0;
    for (uint16_t band = 0; band < rows; band++)
    {
      uint32_t shots = shotsInBand(band);
      if (shots > maximum) maximum = shots;
    }
    if (safety.validateTarget(startPanMdeg, startTiltMdeg) == SAFETY_TILT_LIMIT ||
        safety.validateTarget(endPanMdeg, endTiltMdeg) == SAFETY_TILT_LIMIT) return "TILT_LIMIT";
    if (safety.validateTarget(startPanMdeg, startTiltMdeg) == SAFETY_PAN_LIMIT ||
        safety.validateTarget(endPanMdeg, endTiltMdeg) == SAFETY_PAN_LIMIT) return "PAN_LIMIT";
    return maximum == columns ? "OK" : "VALUE_OUT_OF_RANGE";
  }
  if (panSpeedPermille == 0 || panSpeedPermille > 2000 ||
      tiltSpeedPermille == 0 || tiltSpeedPermille > 2000) return "VALUE_OUT_OF_RANGE";
  if (!timingValid(settleTimeMs, triggerPulseMs, exposureWaitMs)) return "VALUE_OUT_OF_RANGE";
  if (disconnectPolicy == DISCONNECT_ESTOP_ON_TIMEOUT &&
      (heartbeatTimeoutMs < 500UL || heartbeatTimeoutMs > 60000UL)) return "VALUE_OUT_OF_RANGE";

  TargetPositionV2 first;
  TargetPositionV2 lastPan;
  TargetPositionV2 lastTilt;
  if (!targetFor(0, first)) return "VALUE_OUT_OF_RANGE";
  if (!targetFor((uint32_t)(columns - 1) * rows, lastPan)) return "VALUE_OUT_OF_RANGE";
  if (!targetFor(rows - 1, lastTilt)) return "VALUE_OUT_OF_RANGE";
  SafetyResult firstResult = safety.validateTarget(first.panMdeg, first.tiltMdeg);
  SafetyResult panResult = safety.validateTarget(lastPan.panMdeg, first.tiltMdeg);
  SafetyResult tiltResult = safety.validateTarget(first.panMdeg, lastTilt.tiltMdeg);
  if (firstResult == SAFETY_TILT_LIMIT || tiltResult == SAFETY_TILT_LIMIT) return "TILT_LIMIT";
  if (firstResult == SAFETY_PAN_LIMIT || panResult == SAFETY_PAN_LIMIT) return "PAN_LIMIT";
  if (firstResult != SAFETY_OK || panResult != SAFETY_OK || tiltResult != SAFETY_OK) return "SAFETY_UNCONFIRMED";
  return "OK";
}
