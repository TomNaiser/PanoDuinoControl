#ifndef PANORAMA_PLAN_V2_H
#define PANORAMA_PLAN_V2_H

#include <stdint.h>
#include "ProtocolMessages.h"
#include "SafetyController.h"

struct TargetPositionV2
{
  uint32_t imageIndex;
  uint16_t column;
  uint16_t row;
  int32_t panMdeg;
  int32_t tiltMdeg;
};

struct PanoramaPlanV2
{
  uint32_t runId;
  uint16_t planVersion;
  uint8_t moveMode;
  int32_t startPanMdeg;
  int32_t startTiltMdeg;
  uint16_t columns;
  uint16_t rows;
  int32_t stepPanMdeg;
  int32_t stepTiltMdeg;
  int32_t endPanMdeg;
  int32_t endTiltMdeg;
  uint32_t settleTimeMs;
  uint32_t triggerPulseMs;
  uint32_t exposureWaitMs;
  uint16_t panSpeedPermille;
  uint16_t tiltSpeedPermille;
  DisconnectPolicyV2 disconnectPolicy;
  uint32_t heartbeatTimeoutMs;

  PanoramaPlanV2();
  uint32_t totalImages() const;
  bool targetFor(uint32_t imageIndex, TargetPositionV2 &target) const;
  uint32_t shotsInBand(uint16_t band) const;
  static bool timingValid(uint32_t settleMs, uint32_t triggerMs, uint32_t exposureMs);
  const char *validate(const SafetyController &safety) const;
};

#endif
