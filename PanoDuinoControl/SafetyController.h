#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#include <stdint.h>
#include "SafetyConfiguration.h"

enum SafetyResult
{
  SAFETY_OK,
  SAFETY_LIMITS_NOT_CONFIRMED,
  SAFETY_PAN_LIMIT,
  SAFETY_TILT_LIMIT
};

class SafetyController
{
public:
  SafetyController();
  SafetyResult validateTarget(int32_t panMdeg, int32_t tiltMdeg) const;
  SafetyResult validateManualTarget(int32_t actualPan, int32_t actualTilt, int32_t targetPan, int32_t targetTilt) const;
  int limitDrive(bool tiltAxis, int32_t positionMdeg, int signedPwm) const;
  SafetyResult observeTilt(int32_t tiltMdeg);
  bool limitsConfirmed() const;
  bool motionEnabled() const;
  void enableMotion();
  void stopMotion();
  const char *lastFault() const;
  void loadTiltLimits();
  bool setTiltLimits(int32_t minimumMdeg, int32_t maximumMdeg);
  bool setTiltZeroOffsetMdeg(int32_t offsetMdeg);
  int32_t tiltZeroOffsetMdeg() const { return _tiltZeroOffsetMdeg; }
  void beginBootSession();
  uint32_t bootId() const { return _bootId; }

  int32_t panMinimumMdeg() const { return PAN_HARD_MIN_MDEG; }
  int32_t panMaximumMdeg() const { return PAN_HARD_MAX_MDEG; }
  int32_t tiltMinimumMdeg() const { return _tiltMinimumMdeg - _tiltZeroOffsetMdeg; }
  int32_t tiltMaximumMdeg() const { return _tiltMaximumMdeg - _tiltZeroOffsetMdeg; }

private:
  bool _motionEnabled;
  const char *_lastFault;
  int32_t _tiltMinimumMdeg;
  int32_t _tiltMaximumMdeg;
  int32_t _tiltZeroOffsetMdeg;
  uint32_t _bootId;
};

#endif
