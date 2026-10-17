#ifndef V2_CONTROLLER_H
#define V2_CONTROLLER_H

#include <Arduino.h>
#include "ProtocolV2.h"
#include "PanoramaExecutor.h"
#include "DebugEventLog.h"
#include "StackWatermark.h"

struct ManualCaptureRecordV2
{
  uint32_t eventSequence;
  uint16_t captureSequence;
  uint16_t rasterCaptureCount;
  int32_t panMdeg;
  int32_t tiltMdeg;
  uint32_t deviceTimeMs;
  bool attachedToRun;
};

class V2Controller : public IV2FrameHandler, public IExecutorEvents
{
public:
  V2Controller(Stream &stream, IPanoramaHardware &hardware);
  void begin(uint32_t nowMs);
  void tick(uint32_t nowMs);
  bool isReceiving() const { return _protocol.isReceiving(); }
  bool sessionActive() const { return _sessionActive; }
  bool motionEnabled() const { return _safety.motionEnabled(); }
  int limitDrive(bool tiltAxis, int32_t positionMdeg, int signedPwm) const { return _safety.limitDrive(tiltAxis, positionMdeg, signedPwm); }
  SafetyResult validateLegacyTarget(int32_t panMdeg, int32_t tiltMdeg);
  void observeTilt(int32_t tiltMdeg);
  void emergencyStop(const char *reason);
  void fault(const char *reason);

  virtual void handleV2Frame(const V2Frame &frame, ProtocolV2 &protocol, uint32_t nowMs);
  virtual void onStateChanged(RunStateV2 state);
  virtual void onPhaseChanged(RunPhaseV2 previousPhase, RunPhaseV2 phase);
  virtual void onImageCompleted(const TargetPositionV2 &target, int32_t actualPanMdeg, int32_t actualTiltMdeg,
                                uint32_t captureSequence, uint16_t planRevision);

private:
  enum ManualKind : uint8_t { MANUAL_NONE, MANUAL_MOVE, MANUAL_TRIGGER, MANUAL_JOG };
  bool _jogActive=false;
  uint32_t _jogToken=0, _lastStoppedJogToken=0, _jogUpdatedMs=0;
  void tickJog(uint32_t nowMs);
  enum ManualResult : uint8_t { MANUAL_IDLE, MANUAL_ACTIVE, MANUAL_REACHED, MANUAL_TRIGGERED, MANUAL_CANCELLED, MANUAL_TIMED_OUT, MANUAL_FAULT, MANUAL_MISSED };
  bool parsePlan(const V2Frame &frame, PanoramaPlanV2 &plan, const char *&errorCode);
  void sendCapabilities();
  void sendDeviceInfo();
  void sendDiagnostics(uint32_t nowMs);
  #ifndef __AVR__
  void sendDebugLog(uint16_t afterSequence);
  void sendDebugRecord(const DebugEventRecordV2 &record);
  void recordDebug(DebugEventCodeV2 code);
  void flushDebugLog();
  #else
  void recordDebug(DebugEventCodeV2) {}
  #endif
  void sendPlan();
  void sendSegments();
  void sendSegment(uint8_t index);
  void sendManualRecord(const ManualCaptureRecordV2 &record, bool replayed);
  void addRunStateField(RunStateV2 state);
  void addRunPhaseField(RunPhaseV2 phase);
  void sendStatus();
  void sendManualStatus();
  void finishManualStatus(ManualResult result);
  void checkHeartbeat(uint32_t nowMs);
  void tickManualTrigger(uint32_t nowMs);
  void tickTimelapse(uint32_t nowMs);
  void stopTimelapse();
  void tickManualMove(uint32_t nowMs);
  void tickBenchDrive(uint32_t nowMs);
  void sendBenchStatus();
  void sendManualImageTriggered(uint32_t nowMs);
  bool parseUInt32(const char *value, uint32_t &result) const;
  bool parseUInt16(const char *value, uint16_t &result) const;
  bool parseInt32(const char *value, int32_t &result) const;
  bool requireRunId(const V2Frame &frame, uint32_t &runId, ProtocolV2 &protocol);
  bool handleMotionConfiguration(const V2Frame &frame,ProtocolV2 &protocol,uint32_t nowMs);
  bool configurationWritable() const;
  void sendMotionConfigurationInfo();

  IPanoramaHardware &_hardware;
  SafetyController _safety;
  ProtocolV2 _protocol;
  PanoramaExecutor _executor;
  bool _sessionActive;
  uint32_t _lastHeartbeatMs;
  uint32_t _lastStatusMs;
  uint32_t _eventSequence;
  bool _manualTriggerActive;
  uint32_t _manualTriggerEndMs;
  bool _timelapseActive;
  bool _timelapsePulseActive;
  uint32_t _timelapseRunId;
  uint32_t _timelapseIntervalMs;
  uint32_t _timelapseNextMs;
  uint32_t _timelapsePulseEndMs;
  uint32_t _timelapseCaptureCount;
  uint16_t _timelapsePulseMs;
  bool _manualMoveActive;
  bool _benchActive = false;
  int32_t _manualTargetPan, _manualTargetTilt;
  uint32_t _manualMoveStartedMs;
  uint32_t _manualMoveMaximumMs;
  bool _manualIncremental;
  uint16_t _manualCommand = 0;
  ManualKind _manualKind = MANUAL_NONE;
  ManualResult _manualResult = MANUAL_IDLE;
  int32_t _manualStatusPan = 0, _manualStatusTilt = 0;
  uint32_t _manualStatusStartedMs = 0, _manualStatusDurationMs = 0, _manualStatusMaximumMs = 0;
  uint16_t _manualCaptureSequence;
  uint32_t _currentTickMs;
  RunStateV2 _lastObservedState;
  #ifndef __AVR__
  DebugEventLogV2 _debugLog;
  DebugEventRecordV2 _pendingDebug[DEBUG_EVENT_CAPACITY_V2];
  uint8_t _pendingDebugCount = 0, _pendingDebugOffset = 0;
  uint16_t _pendingDebugNewest = 0, _pendingDebugDropped = 0;
  bool _pendingDebugActive = false;
  #endif
  StackWatermark _stackWatermark;
  MotionSettings _stagedMotion;
  uint32_t _configToken=0, _configRevision=0, _configStartedMs=0, _configUpdatedMs=0;
  uint32_t _lastAppliedConfigToken=0, _lastAppliedConfigBase=0;
  uint8_t _configSeen[(MOTION_PARAMETER_COUNT+7)/8]={0};
};

#endif
