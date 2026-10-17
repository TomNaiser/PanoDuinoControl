#ifndef PANORAMA_EXECUTOR_H
#define PANORAMA_EXECUTOR_H

#include <stdint.h>
#include "PanoramaPlanV2.h"
#include "ProtocolMessages.h"
#include "MotionConfiguration.h"
#include "BenchDrive.h"
#include "MotionSettings.h"

struct MotionDiagnosticsV2
{
  uint32_t sampleSequence;
  uint8_t stableSamples;
  uint8_t requiredStableSamples;
  bool panWithinTolerance;
  bool tiltWithinTolerance;
  int32_t panTargetMdeg;
  int32_t tiltTargetMdeg;
  int32_t panErrorMdeg;
  int32_t tiltErrorMdeg;
};

struct MotionTelemetryV2
{
  int32_t panReferenceMdeg, tiltReferenceMdeg;
  int32_t panVelocityMdegPerSecond, tiltVelocityMdegPerSecond;
  int32_t panReferenceVelocity, tiltReferenceVelocity;
  int32_t panSignedPwm, tiltSignedPwm;
  uint8_t panPhase, tiltPhase;
  uint8_t panDriveStage, tiltDriveStage;
};

class IPanoramaHardware
{
public:
  virtual ~IPanoramaHardware() {}
  virtual bool setTargetMdeg(int32_t panMdeg, int32_t tiltMdeg, uint16_t panSpeedPermille, uint16_t tiltSpeedPermille, bool requireMeasuredProgress) = 0;
  virtual bool setManualTargetMdeg(int32_t pan,int32_t tilt,uint16_t panSpeed,uint16_t tiltSpeed)
  { return setTargetMdeg(pan,tilt,panSpeed,tiltSpeed,true); }
  virtual bool setPositioningTargetMdeg(int32_t pan,int32_t tilt,uint16_t panSpeed,uint16_t tiltSpeed)
  { return setTargetMdeg(pan,tilt,panSpeed,tiltSpeed,false); }
  virtual bool setColumnReturnTargetMdeg(int32_t pan,int32_t tilt,uint16_t panSpeed,uint16_t tiltSpeed)
  { return setPositioningTargetMdeg(pan,tilt,panSpeed,tiltSpeed); }
  virtual uint16_t columnReturnTiltTempo(uint16_t rowTempo) const { return rowTempo; }
  virtual uint32_t incrementalMoveMaximumDurationMs() const { return PANORAMA_MOVEMENT_TIMEOUT_MS; }
  virtual uint32_t positioningMoveMaximumDurationMs() const { return 0; }
  virtual bool supportsManualJog() const { return false; }
  virtual bool startManualJog(bool, int, uint16_t) { return false; }
    virtual uint16_t manualJogMaximumVelocityMdeg() const { return 0; }
    virtual uint16_t manualJogMaximumPanVelocityMdeg() const { return manualJogMaximumVelocityMdeg(); }
    virtual bool startManualJogVelocity(bool, int, uint16_t) { return false; }
  virtual bool manualJogLimitReached() const { return false; }
  virtual uint32_t manualJogLeaseMs() const { return 750; }
  virtual bool targetsReached() = 0;
  virtual bool exposurePositionStable(int32_t pan, int32_t tilt) { return stoppedPositionStable(pan,tilt); }
  // Passive feedback only: never request a new target after a timed-out step.
  virtual bool stoppedPositionResting() const { return false; }
  virtual bool stoppedPositionStable(int32_t, int32_t) const { return false; }
  virtual void setCameraActive(bool active) = 0;
  virtual void stopAllMotion() = 0;
  virtual int32_t actualPanMdeg() const = 0;
  virtual int32_t actualTiltMdeg() const = 0;
  virtual bool setTiltZeroOffsetMdeg(int32_t) { return false; }
  virtual void getMotionDiagnostics(MotionDiagnosticsV2 &diagnostics) const = 0;
  // Bits 1/2: raw capture error exceeds the configured Pan/Tilt tolerance.
  virtual uint8_t captureTargetErrorFlags(int32_t, int32_t) const { return 0; }
  virtual void configureMotionLimits(int32_t, int32_t, int32_t, int32_t) {}
  virtual bool targetQuietSince(uint32_t &) const { return false; }
  virtual const char *motionFault() const { return 0; }
  virtual bool getMotionTelemetry(MotionTelemetryV2 &) const { return false; }
  virtual bool getMotionConfiguration(MotionSettings &,uint32_t &) const { return false; }
  virtual void loadMotionConfiguration() {}
  virtual const char *applyMotionConfiguration(const MotionSettings &,uint32_t) { return "CONFIG_UNSUPPORTED"; }
  // Normal hardware does not expose unregulated actuator commands.
  virtual bool startBenchDrive(bool, int, uint32_t, int32_t, uint32_t) { return false; }
  virtual bool startBenchCruise(bool, int, uint32_t, int32_t, uint32_t) { return false; }
  virtual bool getBenchStatus(BenchDriveStatus &) const { return false; }
  virtual bool getBenchSample(uint16_t, BenchDriveSample &) const { return false; }
  virtual bool getBenchOutput(BenchOutputStatus &) const { return false; }
};

class IExecutorEvents
{
public:
  virtual ~IExecutorEvents() {}
  virtual void onStateChanged(RunStateV2 state) = 0;
  virtual void onPhaseChanged(RunPhaseV2 previousPhase, RunPhaseV2 phase) = 0;
  virtual void onImageCompleted(const TargetPositionV2 &target, int32_t actualPanMdeg, int32_t actualTiltMdeg,
                                uint32_t captureSequence, uint16_t planRevision) = 0;
};

// Ein Segment entsteht beim Start und nach jedem Spaltenwechsel. Fuenf Segmente
// erlauben vier Spruenge, ohne unvertretbar viel Nano-SRAM zu binden.
const uint8_t MAX_RUN_SEGMENTS_V2 = 5;

struct RunSegmentV2
{
  uint16_t revision;
  uint16_t startImageIndex;
  uint16_t captureCount;
};

class PanoramaExecutor
{
public:
  PanoramaExecutor(IPanoramaHardware &hardware, SafetyController &safety, IExecutorEvents &events);
  const char *loadPlan(const PanoramaPlanV2 &plan);
  const char *start(uint32_t runId, uint32_t nowMs);
  const char *pause(uint32_t runId);
  const char *resume(uint32_t runId);
  const char *rewindColumns(uint32_t runId, uint16_t columns);
  const char *forwardColumns(uint32_t runId, uint16_t columns);
  const char *updateTiming(uint32_t runId, uint32_t settleMs, uint32_t triggerMs, uint32_t exposureMs);
  const char *stop(uint32_t runId, const char *reason);
  void emergencyStop(const char *reason);
  void fault(const char *reason);
  bool stopMissedTarget(const char *reason,uint32_t nowMs);
  void clearRun();
  void tick(uint32_t nowMs);

  RunStateV2 state() const { return _state; }
  RunPhaseV2 phase() const { return _phase; }
  const PanoramaPlanV2 &plan() const { return _plan; }
  const TargetPositionV2 &currentTarget() const { return _target; }
  uint32_t completedImages() const { return _captureCount; }
  uint32_t nextImageIndex() const { return _nextImageIndex; }
  uint16_t planRevision() const { return _planRevision; }
  uint8_t segmentCount() const { return _segmentCount; }
  const RunSegmentV2 &segment(uint8_t index) const { return _segments[index]; }
  bool pauseRequested() const { return _pauseRequested; }
  uint16_t pendingRewindColumns() const { return _pendingRewindColumns; }
  uint16_t pendingForwardColumns() const { return _pendingForwardColumns; }
  bool lastNavigationForward() const { return _lastNavigationForward; }
  uint16_t timingRevision() const { return _timingRevision; }
  const char *lastError() const { return _lastError; }
  uint32_t phaseStartedMs() const { return _phaseStartedMs; }
  uint32_t movementStartedMs() const { return _movementStartedMs; }
  uint32_t activeMoveMaximumDurationMs() const {
    return (_state==RUN_RUNNING || _state==RUN_PAUSE_PENDING) && (_phase==PHASE_MOVING || _phase==PHASE_SETTLING) ? _incrementalMoveBudgetMs : 0;
  }

private:
  void transition(RunStateV2 state, RunPhaseV2 phase);
  bool selectCurrentTarget();
  void applyRewindColumns(uint16_t columns);
  void applyForwardColumns(uint16_t columns);
  void applyTiming(uint32_t settleMs, uint32_t triggerMs, uint32_t exposureMs);
  bool elapsed(uint32_t nowMs, uint32_t durationMs) const;

  IPanoramaHardware &_hardware;
  SafetyController &_safety;
  IExecutorEvents &_events;
  PanoramaPlanV2 _plan;
  TargetPositionV2 _target;
  RunStateV2 _state;
  RunPhaseV2 _phase;
  uint32_t _phaseStartedMs;
  uint32_t _movementStartedMs;
  uint32_t _incrementalMoveBudgetMs;
  bool _stepTimedOut, _passiveSettling;
  int32_t _capturePanMdeg, _captureTiltMdeg;
  uint32_t _nextImageIndex;
  uint32_t _captureCount;
  uint16_t _planRevision;
  RunSegmentV2 _segments[MAX_RUN_SEGMENTS_V2];
  uint8_t _segmentCount;
  bool _pauseRequested;
  uint16_t _pendingRewindColumns;
  uint16_t _pendingForwardColumns;
  bool _lastNavigationForward;
  uint32_t _pendingSettleMs;
  uint32_t _pendingTriggerMs;
  uint32_t _pendingExposureMs;
  bool _timingPending;
  uint16_t _timingRevision;
  char _lastError[32];
};

#endif
