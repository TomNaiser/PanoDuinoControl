#include "V2Controller.h"
#include <limits.h>
#include <string.h>
#include "VersionInfo.h"

#ifdef __AVR__
#define V2_FLASH(value) F(value)
#define V2_MESSAGE_IS(value, literal) (strcmp_P((value), PSTR(literal)) == 0)
#else
#define V2_FLASH(value) value
#define V2_MESSAGE_IS(value, literal) (strcmp((value), (literal)) == 0)
#endif

static const uint32_t STATUS_INTERVAL_MS = 500;
static const uint32_t MANUAL_MOVEMENT_TIMEOUT_MS = 120000UL;

V2Controller::V2Controller(Stream &stream, IPanoramaHardware &hardware)
  : _hardware(hardware), _protocol(stream, *this),
    _executor(hardware, _safety, *this), _sessionActive(false),
    _lastHeartbeatMs(0), _lastStatusMs(0), _eventSequence(0),
    _manualTriggerActive(false), _manualTriggerEndMs(0),
    _timelapseActive(false), _timelapsePulseActive(false), _timelapseRunId(0),
    _timelapseIntervalMs(0), _timelapseNextMs(0), _timelapsePulseEndMs(0),
    _timelapseCaptureCount(0), _timelapsePulseMs(0),
    _manualMoveActive(false), _manualTargetPan(0), _manualTargetTilt(0), _manualMoveStartedMs(0),
    _manualMoveMaximumMs(MANUAL_MOVEMENT_TIMEOUT_MS),_manualIncremental(false),_manualCaptureSequence(0),
    _currentTickMs(0), _lastObservedState(RUN_IDLE)
{
}

void V2Controller::begin(uint32_t nowMs)
{
  _currentTickMs = nowMs;
  _lastHeartbeatMs = nowMs;
  _lastStatusMs = nowMs;
  _safety.stopMotion();
  _hardware.stopAllMotion();
  _hardware.setCameraActive(false);
  _safety.loadTiltLimits();
  _hardware.setTiltZeroOffsetMdeg(_safety.tiltZeroOffsetMdeg());
  _hardware.loadMotionConfiguration();
  _hardware.configureMotionLimits(_safety.panMinimumMdeg(), _safety.panMaximumMdeg(),
    _safety.tiltMinimumMdeg(), _safety.tiltMaximumMdeg());
  _safety.beginBootSession();
  // Erst nach der Hardwareinitialisierung aufgerufen: Der markierte Bereich
  // repraesentiert den fuer den regulaeren Betrieb verfuegbaren Stackraum.
  _stackWatermark.begin();
  recordDebug(DEBUG_BOOT);
}

void V2Controller::tick(uint32_t nowMs)
{
  if(_configToken && (uint32_t(nowMs-_configUpdatedMs)>15000UL || uint32_t(nowMs-_configStartedMs)>120000UL))
    _configToken=0; // Expiry only discards staging; never activates values.
  _currentTickMs = nowMs;
  _protocol.tick(nowMs, !_sessionActive);
  tickJog(nowMs);
  tickManualTrigger(nowMs);
  tickTimelapse(nowMs);
  checkHeartbeat(nowMs);
  uint16_t revisionBeforeTick = _executor.planRevision();
  uint16_t timingRevisionBeforeTick = _executor.timingRevision();
  _executor.tick(nowMs);
  if (_executor.planRevision() != revisionBeforeTick)
  {
    recordDebug(_executor.lastNavigationForward() ? DEBUG_FORWARDED : DEBUG_REWOUND);
    sendSegments();
    sendStatus();
  }
  if (_executor.timingRevision() != timingRevisionBeforeTick)
  {
    sendPlan();
    sendStatus();
  }
  tickManualMove(nowMs);
  tickBenchDrive(nowMs);
  #ifndef __AVR__
  flushDebugLog();
  #endif
  if (_sessionActive && (uint32_t)(nowMs - _lastStatusMs) >= STATUS_INTERVAL_MS)
  {
    sendStatus();
    _lastStatusMs = nowMs;
  }
}

SafetyResult V2Controller::validateLegacyTarget(int32_t panMdeg, int32_t tiltMdeg)
{
  int64_t relativeTilt = (int64_t)tiltMdeg - _safety.tiltZeroOffsetMdeg();
  SafetyResult result = relativeTilt < INT32_MIN || relativeTilt > INT32_MAX ? SAFETY_TILT_LIMIT :
    _safety.validateTarget(panMdeg, (int32_t)relativeTilt);
  if (result == SAFETY_OK) _safety.enableMotion();
  else _hardware.stopAllMotion();
  return result;
}

void V2Controller::observeTilt(int32_t tiltMdeg)
{
  if (!_safety.motionEnabled()) return;
  if (_manualMoveActive && _safety.validateManualTarget(_hardware.actualPanMdeg(), tiltMdeg,
      _manualTargetPan, _manualTargetTilt) == SAFETY_OK) return;
  if (_safety.observeTilt(tiltMdeg) != SAFETY_OK)
  {
    _executor.fault("FAULT_TILT_LIMIT");
  }
}

void V2Controller::emergencyStop(const char *reason)
{
  finishManualStatus(MANUAL_CANCELLED);
  _benchActive = false;
  _manualMoveActive = false;
  _manualTriggerActive = false;
  stopTimelapse();
  _executor.emergencyStop(reason);
}

void V2Controller::fault(const char *reason)
{
  if(_executor.stopMissedTarget(reason,_currentTickMs)) return;
  if(_manualMoveActive && reason && !strcmp(reason,"MOTION_TARGET_MISSED") &&
     _safety.validateTarget(_hardware.actualPanMdeg(),_hardware.actualTiltMdeg())==SAFETY_OK)
  {
    _hardware.stopAllMotion(); _safety.stopMotion();
    _manualMoveActive=false; finishManualStatus(MANUAL_MISSED); sendStatus(); return;
  }
  finishManualStatus(MANUAL_FAULT);
  _benchActive = false;
  _manualMoveActive = false;
  _manualTriggerActive = false;
  stopTimelapse();
  _executor.fault(reason);
}

void V2Controller::handleV2Frame(const V2Frame &frame, ProtocolV2 &protocol, uint32_t nowMs)
{
  _currentTickMs = nowMs;
  if (V2_MESSAGE_IS(frame.messageType, "HELLO"))
  {
    if (frame.fieldCount != 1) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    _sessionActive = true;
    _lastHeartbeatMs = nowMs;
    recordDebug(DEBUG_SESSION_OPENED);
    protocol.sendAck(frame.sequence, V2_FLASH("HELLO_ACK"));
    sendDeviceInfo();
    sendCapabilities();
    sendPlan();
    sendSegments();
    sendStatus();
    sendMotionConfigurationInfo();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "HEARTBEAT"))
  {
    // The Bluetooth module can stay connected while only the MCU restarts.
    // Publish the new identity before returning to periodic status traffic.
    if (!_sessionActive) { sendDeviceInfo(); sendCapabilities(); sendStatus(); }
    _sessionActive = true;
    _lastHeartbeatMs = nowMs;
    return;
  }
  if(handleMotionConfiguration(frame,protocol,nowMs)) return;
  if(_jogActive && strncmp(frame.messageType,"GET_",4)!=0 &&
     !V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_KEEPALIVE") && !V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_STOP") &&
     !V2_MESSAGE_IS(frame.messageType,"ESTOP") && !V2_MESSAGE_IS(frame.messageType,"STOP"))
  { protocol.sendNack(frame.sequence,V2_FLASH("INVALID_STATE")); return; }
  if(_configToken && !V2_MESSAGE_IS(frame.messageType,"ESTOP") &&
     !V2_MESSAGE_IS(frame.messageType,"GET_STATUS") && !V2_MESSAGE_IS(frame.messageType,"GET_DIAGNOSTICS") &&
     !V2_MESSAGE_IS(frame.messageType,"GET_CAPABILITIES") && !V2_MESSAGE_IS(frame.messageType,"CLEAR_RUN"))
  { protocol.sendNack(frame.sequence,V2_FLASH("CONFIG_EDIT_ACTIVE")); return; }
  if (_benchActive && !V2_MESSAGE_IS(frame.messageType,"GET_STATUS") &&
      !V2_MESSAGE_IS(frame.messageType,"GET_CAPABILITIES") && !V2_MESSAGE_IS(frame.messageType,"GET_DIAGNOSTICS") &&
      !V2_MESSAGE_IS(frame.messageType,"GET_SUPPORT_BUNDLE") && !V2_MESSAGE_IS(frame.messageType,"GET_DEBUG_LOG") &&
      !V2_MESSAGE_IS(frame.messageType,"GET_PLAN") && !V2_MESSAGE_IS(frame.messageType,"GET_BENCH_STATUS") &&
      !V2_MESSAGE_IS(frame.messageType,"GET_BENCH_OUTPUT") &&
      !V2_MESSAGE_IS(frame.messageType,"GET_BENCH_TRACE") && !V2_MESSAGE_IS(frame.messageType,"ESTOP") &&
      !V2_MESSAGE_IS(frame.messageType,"STOP"))
  { protocol.sendNack(frame.sequence,V2_FLASH("BENCH_ACTIVE")); return; }
  if (V2_MESSAGE_IS(frame.messageType, "GET_STATUS"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    protocol.sendAck(frame.sequence, V2_FLASH("OK"));
    sendStatus();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "GET_CAPABILITIES"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    protocol.sendAck(frame.sequence, V2_FLASH("OK"));
    sendCapabilities();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "SET_TILT_LIMITS"))
  {
    int32_t minimum, maximum;
    if (frame.fieldCount != 2 || !parseInt32(frame.fields[0], minimum) ||
        !parseInt32(frame.fields[1], maximum))
    { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    if (!(_executor.state() == RUN_IDLE || _executor.state() == RUN_STOPPED ||
          _executor.state() == RUN_COMPLETED) || _manualMoveActive || _manualTriggerActive || _timelapseActive ||
        (_safety.motionEnabled() && !_hardware.targetsReached()))
    { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    if ((int64_t)minimum + _safety.tiltZeroOffsetMdeg() < TILT_ABSOLUTE_MIN_MDEG ||
        (int64_t)maximum + _safety.tiltZeroOffsetMdeg() > TILT_ABSOLUTE_MAX_MDEG || minimum >= maximum ||
        _hardware.actualTiltMdeg() < minimum || _hardware.actualTiltMdeg() > maximum)
    { protocol.sendNack(frame.sequence, V2_FLASH("VALUE_OUT_OF_RANGE")); return; }
    _hardware.stopAllMotion(); _safety.stopMotion();
    if (!_safety.setTiltLimits(minimum, maximum))
    { protocol.sendNack(frame.sequence, V2_FLASH("STORE_FAILED")); return; }
    protocol.sendAck(frame.sequence, V2_FLASH("TILT_LIMITS_SAVED"));
    sendCapabilities();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "SET_TILT_ZERO") || V2_MESSAGE_IS(frame.messageType, "RESET_TILT_ZERO"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    if (_executor.state() != RUN_IDLE || _manualMoveActive || _manualTriggerActive || _timelapseActive ||
        (_safety.motionEnabled() && !_hardware.targetsReached()))
    { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    int32_t actual = _hardware.actualTiltMdeg(), previous = _safety.tiltZeroOffsetMdeg();
    if (actual < _safety.tiltMinimumMdeg() || actual > _safety.tiltMaximumMdeg())
    { protocol.sendNack(frame.sequence, V2_FLASH("TILT_LIMIT")); return; }
    int32_t offset = V2_MESSAGE_IS(frame.messageType, "RESET_TILT_ZERO") ? 0 : actual + previous;
    _hardware.stopAllMotion(); _safety.stopMotion();
    if (!_hardware.setTiltZeroOffsetMdeg(offset))
    { protocol.sendNack(frame.sequence, V2_FLASH("CALIBRATION_UNSUPPORTED")); return; }
    if (!_safety.setTiltZeroOffsetMdeg(offset))
    {
      _hardware.setTiltZeroOffsetMdeg(previous);
      protocol.sendNack(frame.sequence, V2_FLASH("STORE_FAILED")); return;
    }
    protocol.sendAck(frame.sequence, V2_FLASH("TILT_ZERO_SAVED"));
    sendCapabilities(); sendStatus(); return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "GET_DIAGNOSTICS"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    protocol.sendAck(frame.sequence, V2_FLASH("OK"));
    sendDiagnostics(nowMs);
    return;
  }
  #ifndef __AVR__
  if (V2_MESSAGE_IS(frame.messageType, "GET_DEBUG_LOG"))
  {
    uint16_t afterSequence = 0;
    if (frame.fieldCount > 1 || (frame.fieldCount == 1 && !parseUInt16(frame.fields[0], afterSequence)))
    { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    protocol.sendAck(frame.sequence, V2_FLASH("OK"));
    sendDebugLog(afterSequence);
    return;
  }
  #endif
  if (V2_MESSAGE_IS(frame.messageType, "GET_SUPPORT_BUNDLE"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    protocol.sendAck(frame.sequence, V2_FLASH("OK"));
    sendDeviceInfo();
    sendCapabilities();
    sendPlan();
    sendSegments();
    sendStatus();
    sendDiagnostics(nowMs);
    #ifndef __AVR__
    sendDebugLog(0);
    #endif
    return;
  }
  #ifndef __AVR__
  if (V2_MESSAGE_IS(frame.messageType, "CLEAR_DEBUG_LOG"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    _debugLog.clear();
    protocol.sendAck(frame.sequence, V2_FLASH("CLEARED"));
    return;
  }
  #endif
  if (V2_MESSAGE_IS(frame.messageType, "GET_PLAN"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    protocol.sendAck(frame.sequence, V2_FLASH("OK"));
    sendPlan();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "GET_BENCH_STATUS") || V2_MESSAGE_IS(frame.messageType, "GET_BENCH_TRACE"))
  {
    BenchDriveStatus status;
    if (!_hardware.getBenchStatus(status)) { protocol.sendNack(frame.sequence,V2_FLASH("BENCH_UNSUPPORTED")); return; }
    if (V2_MESSAGE_IS(frame.messageType,"GET_BENCH_STATUS"))
    {
      if(frame.fieldCount!=0) { protocol.sendNack(frame.sequence,V2_FLASH("BAD_FIELD_COUNT")); return; }
      protocol.sendAck(frame.sequence,V2_FLASH("OK")); sendBenchStatus(); return;
    }
    uint16_t index;
    if(frame.fieldCount!=1 || !parseUInt16(frame.fields[0],index))
    { protocol.sendNack(frame.sequence,V2_FLASH("BAD_FIELD_COUNT")); return; }
    if(status.active || status.recording) { protocol.sendNack(frame.sequence,V2_FLASH("BENCH_ACTIVE")); return; }
    if(index>=status.count) { protocol.sendNack(frame.sequence,V2_FLASH("VALUE_OUT_OF_RANGE")); return; }
    protocol.sendAck(frame.sequence,V2_FLASH("OK"));
    for(uint16_t offset=0;offset<8 && index+offset<status.count;++offset)
    {
      BenchDriveSample sample;
      if(!_hardware.getBenchSample(index+offset,sample)) break;
      protocol.beginFrame(V2_FLASH("BENCH_SAMPLE"));
      protocol.addField((uint32_t)(index+offset)); protocol.addField(sample.elapsedMs);
      protocol.addSignedField(sample.panMdeg); protocol.addSignedField(sample.tiltMdeg);
      protocol.addSignedField(sample.signedPwm); protocol.addField((uint32_t)sample.stage); protocol.endFrame();
    }
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType,"GET_BENCH_OUTPUT"))
  {
    BenchOutputStatus output;
    if(frame.fieldCount!=0) { protocol.sendNack(frame.sequence,V2_FLASH("BAD_FIELD_COUNT")); return; }
    if(!_hardware.getBenchOutput(output)) { protocol.sendNack(frame.sequence,V2_FLASH("BENCH_UNSUPPORTED")); return; }
    protocol.sendAck(frame.sequence,V2_FLASH("OK"));
    protocol.beginFrame(V2_FLASH("BENCH_OUTPUT"));
    protocol.addSignedField(output.panAppliedPwm); protocol.addSignedField(output.tiltAppliedPwm);
    protocol.addField((uint32_t)output.tiltPositivePin); protocol.addField((uint32_t)output.tiltNegativePin);
    protocol.addField(output.tiltPeriod); protocol.addField(output.tiltDuty); protocol.addField(output.tiltCounter);
    protocol.addField(output.tiltPeripheralMux ? 1UL : 0UL); protocol.endFrame(); return;
  }
  if (V2_MESSAGE_IS(frame.messageType,"BENCH_PULSE") || V2_MESSAGE_IS(frame.messageType,"BENCH_CRUISE"))
  {
    bool cruise=V2_MESSAGE_IS(frame.messageType,"BENCH_CRUISE");
    BenchDriveStatus status;
    if (!_hardware.getBenchStatus(status)) { protocol.sendNack(frame.sequence,V2_FLASH("BENCH_UNSUPPORTED")); return; }
    uint16_t axis; int32_t pwm,travel; uint32_t duration;
    if(frame.fieldCount!=4 || !parseUInt16(frame.fields[0],axis) || !parseInt32(frame.fields[1],pwm) ||
       !parseUInt32(frame.fields[2],duration) || !parseInt32(frame.fields[3],travel) || axis>1 ||
       pwm==0 || pwm < -40 || pwm > 40 || duration<100 || duration>3000 || travel<500 || travel>5000 ||
       ((pwm < -30 || pwm > 30) && (cruise || duration>500)) ||
       (cruise && (duration<600 || travel<1000)))
    { protocol.sendNack(frame.sequence,V2_FLASH("VALUE_OUT_OF_RANGE")); return; }
    if(_executor.state()!=RUN_IDLE || _manualMoveActive || _manualTriggerActive || _timelapseActive ||
       _safety.motionEnabled() || status.active || status.recording)
    { protocol.sendNack(frame.sequence,V2_FLASH("INVALID_STATE")); return; }
    int32_t pan=_hardware.actualPanMdeg(),tilt=_hardware.actualTiltMdeg();
    if(_safety.validateTarget(pan,tilt)!=SAFETY_OK || _safety.validateTarget(
       pan+(axis==0 ? (pwm>0 ? travel : -travel) : 0),
       tilt+(axis==1 ? (pwm>0 ? travel : -travel) : 0))!=SAFETY_OK)
    { protocol.sendNack(frame.sequence,V2_FLASH("BENCH_LIMIT")); return; }
    bool started=cruise ? _hardware.startBenchCruise(axis==1,pwm,duration,travel,nowMs) :
      _hardware.startBenchDrive(axis==1,pwm,duration,travel,nowMs);
    if(!started)
    { protocol.sendNack(frame.sequence,V2_FLASH("BENCH_REJECTED")); return; }
    _benchActive=true; _lastHeartbeatMs=nowMs; _safety.enableMotion();
    protocol.sendAck(frame.sequence,V2_FLASH("BENCH_STARTED")); sendBenchStatus(); return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "LOAD_PLAN"))
  {
    if (_timelapseActive) { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    if (_manualMoveActive || _manualTriggerActive) { protocol.sendNack(frame.sequence, V2_FLASH("MANUAL_ACTION_ACTIVE")); return; }
    PanoramaPlanV2 plan;
    const char *parseError = "OK";
    if (!parsePlan(frame, plan, parseError)) { protocol.sendNack(frame.sequence, parseError); return; }
    const char *result = _executor.loadPlan(plan);
    if (strcmp(result, "PLAN_ACCEPTED") == 0) protocol.sendAck(frame.sequence, result);
    else protocol.sendNack(frame.sequence, result);
    if (strcmp(result, "PLAN_ACCEPTED") == 0)
    {
      _timelapseRunId = 0;
      #ifndef __AVR__
      _debugLog.clear();
      #endif
      recordDebug(DEBUG_PLAN_LOADED);
    }
    return;
  }

  uint32_t runId;
  if (V2_MESSAGE_IS(frame.messageType, "START"))
  {
    if (_timelapseActive) { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    if (!requireRunId(frame, runId, protocol)) return;
    if (_manualMoveActive || _manualTriggerActive) { protocol.sendNack(frame.sequence, V2_FLASH("MANUAL_ACTION_ACTIVE")); return; }
    const char *result = _executor.start(runId, nowMs);
    if (strcmp(result, "STARTED") == 0) protocol.sendAck(frame.sequence, result);
    else protocol.sendNack(frame.sequence, result);
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "PAUSE"))
  {
    if (!requireRunId(frame, runId, protocol)) return;
    const char *result = _executor.pause(runId);
    if (strcmp(result, "PAUSED") == 0 || strcmp(result, "PAUSE_PENDING") == 0) protocol.sendAck(frame.sequence, result);
    else protocol.sendNack(frame.sequence, result);
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "RESUME"))
  {
    if (!requireRunId(frame, runId, protocol)) return;
    if (_manualMoveActive || _manualTriggerActive) { protocol.sendNack(frame.sequence, V2_FLASH("MANUAL_ACTION_ACTIVE")); return; }
    const char *result = _executor.resume(runId);
    if (strcmp(result, "RESUMED") == 0) protocol.sendAck(frame.sequence, result);
    else protocol.sendNack(frame.sequence, result);
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "STOP"))
  {
    if (!requireRunId(frame, runId, protocol)) return;
    const char *result = _executor.stop(runId, "USER_STOP");
    if (strcmp(result, "STOPPED") == 0)
    {
      finishManualStatus(MANUAL_CANCELLED);
      _manualMoveActive = false; _manualTriggerActive = false;
      protocol.sendAck(frame.sequence, result);
    }
    else protocol.sendNack(frame.sequence, result);
    sendStatus();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "ESTOP"))
  {
    if (frame.fieldCount < 1 || !parseUInt32(frame.fields[0], runId)) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    _configToken=0;
    finishManualStatus(MANUAL_CANCELLED);
    _benchActive = false;
    _manualMoveActive = false;
    _manualTriggerActive = false;
    stopTimelapse();
    _executor.emergencyStop(frame.fieldCount > 1 ? frame.fields[1] : "ESTOP");
    protocol.sendAck(frame.sequence, V2_FLASH("STOPPED"));
    sendStatus();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "CLEAR_RUN"))
  {
    if (frame.fieldCount != 0) { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    if (_timelapseActive) { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    const bool hadTimelapse = _timelapseRunId != 0;
    if (hadTimelapse)
    {
      _timelapseRunId = 0;
      _timelapseIntervalMs = 0;
      _timelapsePulseMs = 0;
      _timelapseCaptureCount = 0;
    }
    RunStateV2 before = _executor.state();
    _executor.clearRun();
    if (hadTimelapse || before != _executor.state())
    {
      protocol.sendAck(frame.sequence, V2_FLASH("CLEARED"));
      sendStatus();
    }
    else protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE"));
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "REWIND_COLUMNS"))
  {
    uint16_t columns;
    if (frame.fieldCount != 2 || !parseUInt32(frame.fields[0], runId) || !parseUInt16(frame.fields[1], columns))
    { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    if (_manualMoveActive || _manualTriggerActive) { protocol.sendNack(frame.sequence, V2_FLASH("MANUAL_ACTION_ACTIVE")); return; }
    const char *result = _executor.rewindColumns(runId, columns);
    if (strcmp(result, "REWOUND") == 0)
    {
      protocol.sendAck(frame.sequence, result);
      sendSegments();
      sendStatus();
      recordDebug(DEBUG_REWOUND);
    }
    else if (strcmp(result, "REWIND_PENDING") == 0)
      protocol.sendAck(frame.sequence, result);
    else protocol.sendNack(frame.sequence, result);
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "FORWARD_COLUMNS"))
  {
    uint16_t columns;
    if (frame.fieldCount != 2 || !parseUInt32(frame.fields[0], runId) || !parseUInt16(frame.fields[1], columns))
    { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    if (_manualMoveActive || _manualTriggerActive) { protocol.sendNack(frame.sequence, V2_FLASH("MANUAL_ACTION_ACTIVE")); return; }
    const char *result = _executor.forwardColumns(runId, columns);
    if (strcmp(result, "FORWARDED") == 0)
    {
      protocol.sendAck(frame.sequence, result);
      sendSegments();
      sendStatus();
      recordDebug(DEBUG_FORWARDED);
    }
    else if (strcmp(result, "FORWARD_PENDING") == 0)
      protocol.sendAck(frame.sequence, result);
    else protocol.sendNack(frame.sequence, result);
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "UPDATE_TIMING"))
  {
    uint32_t settleMs;
    uint32_t triggerMs;
    uint32_t exposureMs;
    if (frame.fieldCount != 4 || !parseUInt32(frame.fields[0], runId) ||
        !parseUInt32(frame.fields[1], settleMs) || !parseUInt32(frame.fields[2], triggerMs) ||
        !parseUInt32(frame.fields[3], exposureMs))
    { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    const char *result = _executor.updateTiming(runId, settleMs, triggerMs, exposureMs);
    if (strcmp(result, "TIMING_UPDATED") == 0)
    {
      protocol.sendAck(frame.sequence, result);
      sendPlan();
      sendStatus();
    }
    else if (strcmp(result, "TIMING_QUEUED") == 0)
      protocol.sendAck(frame.sequence, result);
    else protocol.sendNack(frame.sequence, result);
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_STOP") || V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_KEEPALIVE"))
  {
    uint32_t token;
    if(frame.fieldCount!=1 || !parseUInt32(frame.fields[0],token) || !token)
    { protocol.sendNack(frame.sequence,V2_FLASH("BAD_FIELD_COUNT")); return; }
    if(V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_KEEPALIVE"))
    {
      tickJog(nowMs);
      if(!_jogActive || token!=_jogToken) { protocol.sendNack(frame.sequence,V2_FLASH("JOG_RELEASED")); return; }
      _jogUpdatedMs=nowMs; protocol.sendAck(frame.sequence,V2_FLASH("JOG_ALIVE")); return;
    }
    _lastStoppedJogToken=token;
    if(_jogActive && token==_jogToken)
    {
      _hardware.stopAllMotion(); _safety.stopMotion(); finishManualStatus(MANUAL_CANCELLED); sendStatus();
    }
    protocol.sendAck(frame.sequence,V2_FLASH("JOG_STOPPED")); return;
  }
  if(V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_START") || V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_START_VELOCITY"))
  {
    bool velocityMode=V2_MESSAGE_IS(frame.messageType,"MANUAL_JOG_START_VELOCITY");
    uint32_t token; uint16_t axis,tempo; int32_t direction;
    if(frame.fieldCount!=4 || !parseUInt32(frame.fields[0],token) || !token ||
       !parseUInt16(frame.fields[1],axis) || axis>1 || !parseInt32(frame.fields[2],direction) ||
       (direction!=1 && direction!=-1) || !parseUInt16(frame.fields[3],tempo) ||
       (velocityMode ? (tempo<500 || tempo>20000) : (!tempo || tempo>1000)))
    { protocol.sendNack(frame.sequence,V2_FLASH("BAD_FIELD_COUNT")); return; }
    uint16_t maximumVelocity=axis==1 ? _hardware.manualJogMaximumVelocityMdeg() : _hardware.manualJogMaximumPanVelocityMdeg();
    if(velocityMode && (!maximumVelocity || tempo>maximumVelocity))
    { protocol.sendNack(frame.sequence,V2_FLASH("JOG_SPEED_LIMIT")); return; }
    if(token==_lastStoppedJogToken || (_jogActive && token==_jogToken))
    { protocol.sendNack(frame.sequence,V2_FLASH("JOG_RELEASED")); return; }
    if(_jogActive || _manualMoveActive || _manualTriggerActive || _timelapseActive || _benchActive ||
       !(_executor.state()==RUN_IDLE || _executor.state()==RUN_PAUSED || _executor.state()==RUN_PLAN_READY ||
         _executor.state()==RUN_STOPPED || _executor.state()==RUN_COMPLETED))
    { protocol.sendNack(frame.sequence,V2_FLASH("INVALID_STATE")); return; }
    if(_safety.validateTarget(_hardware.actualPanMdeg(),_hardware.actualTiltMdeg())!=SAFETY_OK ||
       !_hardware.supportsManualJog()) { protocol.sendNack(frame.sequence,V2_FLASH("SAFETY_UNCONFIRMED")); return; }
    if(!(velocityMode ? _hardware.startManualJogVelocity(axis==1,direction,tempo) : _hardware.startManualJog(axis==1,direction,tempo)))
    { protocol.sendNack(frame.sequence,V2_FLASH("JOG_NOT_READY")); return; }
    _jogActive=true; _jogToken=token; _jogUpdatedMs=nowMs; _safety.enableMotion();
    _manualCommand=frame.sequence; _manualKind=MANUAL_JOG; _manualResult=MANUAL_ACTIVE;
    _manualStatusPan=_hardware.actualPanMdeg(); _manualStatusTilt=_hardware.actualTiltMdeg();
    _manualStatusStartedMs=nowMs; _manualStatusDurationMs=0; _manualStatusMaximumMs=_hardware.manualJogLeaseMs();
    protocol.sendAck(frame.sequence,V2_FLASH("JOGGING")); sendManualStatus(); return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "MANUAL_MOVE"))
  {
    if(_jogActive) { protocol.sendNack(frame.sequence,V2_FLASH("INVALID_STATE")); return; }
    if (_timelapseActive) { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    int32_t pan;
    int32_t tilt;
    uint16_t panSpeed = MOTION_MANUAL_SPEED_PERMILLE, tiltSpeed = MOTION_MANUAL_SPEED_PERMILLE;
    uint16_t incremental=0;
    if ((frame.fieldCount != 2 && frame.fieldCount != 4 && frame.fieldCount != 5) || !parseInt32(frame.fields[0], pan) || !parseInt32(frame.fields[1], tilt) ||
        (frame.fieldCount >= 4 && (!parseUInt16(frame.fields[2], panSpeed) || !parseUInt16(frame.fields[3], tiltSpeed) ||
         panSpeed == 0 || tiltSpeed == 0 || panSpeed > 1000 || tiltSpeed > 1000)) ||
        (frame.fieldCount==5 && (!parseUInt16(frame.fields[4],incremental) || incremental>1)))
    { protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT")); return; }
    if (!(_executor.state() == RUN_IDLE || _executor.state() == RUN_PAUSED ||
          _executor.state() == RUN_PLAN_READY || _executor.state() == RUN_STOPPED ||
          _executor.state() == RUN_COMPLETED) || _manualTriggerActive)
    { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    SafetyResult safetyResult = _safety.validateManualTarget(_hardware.actualPanMdeg(), _hardware.actualTiltMdeg(), pan, tilt);
    if (safetyResult == SAFETY_LIMITS_NOT_CONFIRMED) { protocol.sendNack(frame.sequence, V2_FLASH("SAFETY_UNCONFIRMED")); return; }
    if (safetyResult == SAFETY_TILT_LIMIT) { protocol.sendNack(frame.sequence, V2_FLASH("TILT_LIMIT")); return; }
    if (safetyResult == SAFETY_PAN_LIMIT) { protocol.sendNack(frame.sequence, V2_FLASH("PAN_LIMIT")); return; }
    if (!(incremental==1 ? _hardware.setTargetMdeg(pan,tilt,panSpeed,tiltSpeed,true) :
          _hardware.setManualTargetMdeg(pan,tilt,panSpeed,tiltSpeed))) { protocol.sendNack(frame.sequence, V2_FLASH("TARGET_REJECTED")); return; }
    _manualTargetPan = pan; _manualTargetTilt = tilt;
    _safety.enableMotion();
    _manualMoveActive = true;
    _manualMoveStartedMs = nowMs;
    _manualIncremental=incremental==1;
    _manualMoveMaximumMs=_manualIncremental ? _hardware.incrementalMoveMaximumDurationMs() : MANUAL_MOVEMENT_TIMEOUT_MS;
    _manualCommand=frame.sequence; _manualKind=MANUAL_MOVE; _manualResult=MANUAL_ACTIVE;
    _manualStatusPan=pan; _manualStatusTilt=tilt;
    _manualStatusStartedMs=nowMs; _manualStatusDurationMs=0; _manualStatusMaximumMs=_manualMoveMaximumMs;
    recordDebug(DEBUG_MANUAL_MOVE_STARTED);
    protocol.sendAck(frame.sequence, V2_FLASH("MOVING"));
    sendManualStatus();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "TRIGGER_ONCE"))
  {
    if (_timelapseActive) { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    uint32_t pulseTime;
    if (frame.fieldCount != 1 || !parseUInt32(frame.fields[0], pulseTime) || pulseTime == 0 || pulseTime > 60000UL)
    { protocol.sendNack(frame.sequence, V2_FLASH("VALUE_OUT_OF_RANGE")); return; }
    if (!(_executor.state() == RUN_IDLE || _executor.state() == RUN_PAUSED) || _manualTriggerActive || _manualMoveActive)
    { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    _hardware.setCameraActive(true);
    _manualTriggerActive = true;
    _manualTriggerEndMs = nowMs + pulseTime;
    _manualCommand=frame.sequence; _manualKind=MANUAL_TRIGGER; _manualResult=MANUAL_ACTIVE;
    _manualStatusPan=_hardware.actualPanMdeg(); _manualStatusTilt=_hardware.actualTiltMdeg();
    _manualStatusStartedMs=nowMs; _manualStatusDurationMs=0; _manualStatusMaximumMs=pulseTime;
    protocol.sendAck(frame.sequence, V2_FLASH("TRIGGERED"));
    sendManualStatus();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "START_TIMELAPSE"))
  {
    uint32_t intervalMs;
    uint32_t pulseMs;
    if (frame.fieldCount != 3 || !parseUInt32(frame.fields[0], runId) || runId == 0 ||
        !parseUInt32(frame.fields[1], intervalMs) || !parseUInt32(frame.fields[2], pulseMs) ||
        intervalMs < 100UL || intervalMs > 3600000UL || pulseMs == 0 ||
        pulseMs > 60000UL || pulseMs >= intervalMs)
    { protocol.sendNack(frame.sequence, V2_FLASH("VALUE_OUT_OF_RANGE")); return; }
    if (_executor.state() != RUN_IDLE || _timelapseActive || _manualTriggerActive || _manualMoveActive)
    { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    _timelapseRunId = runId;
    _timelapseIntervalMs = intervalMs;
    _timelapsePulseMs = (uint16_t)pulseMs;
    _timelapseCaptureCount = 0;
    _timelapseNextMs = nowMs + intervalMs;
    _timelapseActive = true;
    protocol.sendAck(frame.sequence, V2_FLASH("TIMELAPSE_STARTED"));
    sendStatus();
    return;
  }
  if (V2_MESSAGE_IS(frame.messageType, "STOP_TIMELAPSE"))
  {
    if (!requireRunId(frame, runId, protocol)) return;
    if (!_timelapseActive || runId != _timelapseRunId)
    { protocol.sendNack(frame.sequence, V2_FLASH("INVALID_STATE")); return; }
    stopTimelapse();
    protocol.sendAck(frame.sequence, V2_FLASH("TIMELAPSE_STOPPED"));
    sendStatus();
    return;
  }
  protocol.sendNack(frame.sequence, V2_FLASH("UNSUPPORTED_MESSAGE"));
}

void V2Controller::onStateChanged(RunStateV2 state)
{
  if(state==RUN_FAULT || state==RUN_STOPPED)
  {
    finishManualStatus(state==RUN_FAULT ? MANUAL_FAULT : MANUAL_CANCELLED);
    _manualMoveActive=false; _manualTriggerActive=false;
  }
  if (state == RUN_RUNNING)
  {
    recordDebug(_lastObservedState == RUN_PAUSED ? DEBUG_RESUMED : DEBUG_RUN_STARTED);
  }
  else if (state == RUN_PAUSE_PENDING) recordDebug(DEBUG_PAUSE_PENDING);
  else if (state == RUN_PAUSED) recordDebug(DEBUG_PAUSED);
  else if (state == RUN_STOPPED) recordDebug(DEBUG_STOPPED);
  else if (state == RUN_COMPLETED) recordDebug(DEBUG_COMPLETED);
  else if (state == RUN_FAULT) recordDebug(DEBUG_FAULT);
  _lastObservedState = state;
  _eventSequence++;
  _protocol.beginFrame(V2_FLASH("STATE_CHANGED"));
  _protocol.addField(_eventSequence);
  addRunStateField(state);
  _protocol.endFrame();
}

void V2Controller::onPhaseChanged(RunPhaseV2 previousPhase, RunPhaseV2 phase)
{
  if (previousPhase == PHASE_SETTLING && phase == PHASE_MOVING)
    recordDebug(DEBUG_SETTLING_LOST);
  else if (phase == PHASE_MOVING)
    recordDebug(DEBUG_TARGET_SELECTED);
  else if (phase == PHASE_SETTLING)
    recordDebug(strcmp(_executor.lastError(),"STEP_MOVE_TIMEOUT")==0 ? DEBUG_STEP_MOVE_TIMEOUT :
      strcmp(_executor.lastError(),"STEP_TARGET_MISSED")==0 ? DEBUG_STEP_TARGET_MISSED : DEBUG_TARGET_REACHED);
  else if (phase == PHASE_TRIGGER_ACTIVE)
    recordDebug(DEBUG_TRIGGER_STARTED);
}

void V2Controller::onImageCompleted(const TargetPositionV2 &target, int32_t actualPanMdeg, int32_t actualTiltMdeg,
                                    uint32_t captureSequence, uint16_t planRevision)
{
  recordDebug(DEBUG_IMAGE_COMMITTED);
  uint8_t warning=_hardware.captureTargetErrorFlags(target.panMdeg,target.tiltMdeg);
  if(!strcmp(_executor.lastError(),"STEP_MOVE_TIMEOUT")) warning|=4;
  if(!strcmp(_executor.lastError(),"STEP_TARGET_MISSED")) warning|=8;
  // Separate additive event keeps the existing IMAGE_COMPLETED wire contract.
  // The app persists this per capture even if subsequent STATUS clears it.
  if(warning)
  {
    _protocol.beginFrame(V2_FLASH("CAPTURE_WARNING"));
    _protocol.addField(_executor.plan().runId);
    _protocol.addField(captureSequence);
    _protocol.addField((uint32_t)warning);
    _protocol.endFrame();
  }
  _eventSequence++;
  _protocol.beginFrame(V2_FLASH("IMAGE_COMPLETED"));
  _protocol.addField(_eventSequence);
  _protocol.addField(_executor.plan().runId);
  _protocol.addField(captureSequence);
  _protocol.addField((uint32_t)planRevision);
  _protocol.addField(target.imageIndex);
  _protocol.addField((uint32_t)target.column);
  _protocol.addField((uint32_t)target.row);
  _protocol.addSignedField(target.panMdeg);
  _protocol.addSignedField(target.tiltMdeg);
  _protocol.addSignedField(actualPanMdeg);
  _protocol.addSignedField(actualTiltMdeg);
  _protocol.addField((uint32_t)0);
  _protocol.endFrame();
}

bool V2Controller::parsePlan(const V2Frame &frame, PanoramaPlanV2 &plan, const char *&errorCode)
{
  if (frame.fieldCount != 16 && frame.fieldCount != 18) { errorCode = "BAD_FIELD_COUNT"; return false; }
  uint16_t planVersion;
  uint16_t moveMode;
  uint16_t disconnectPolicy;
  if (!parseUInt32(frame.fields[0], plan.runId) ||
      !parseUInt16(frame.fields[1], planVersion) ||
      !parseUInt16(frame.fields[2], moveMode) ||
      !parseInt32(frame.fields[3], plan.startPanMdeg) ||
      !parseInt32(frame.fields[4], plan.startTiltMdeg) ||
      !parseUInt16(frame.fields[5], plan.columns) ||
      !parseUInt16(frame.fields[6], plan.rows) ||
      !parseInt32(frame.fields[7], plan.stepPanMdeg) ||
      !parseInt32(frame.fields[8], plan.stepTiltMdeg) ||
      !parseUInt32(frame.fields[9], plan.settleTimeMs) ||
      !parseUInt32(frame.fields[10], plan.triggerPulseMs) ||
      !parseUInt32(frame.fields[11], plan.exposureWaitMs) ||
      !parseUInt16(frame.fields[12], plan.panSpeedPermille) ||
      !parseUInt16(frame.fields[13], plan.tiltSpeedPermille) ||
      !parseUInt16(frame.fields[14], disconnectPolicy) ||
      !parseUInt32(frame.fields[15], plan.heartbeatTimeoutMs))
  {
    errorCode = "VALUE_OUT_OF_RANGE";
    return false;
  }
  if (moveMode > 255 || disconnectPolicy > 1) { errorCode = "VALUE_OUT_OF_RANGE"; return false; }
  if ((moveMode == 1 && frame.fieldCount != 18) || (moveMode != 1 && frame.fieldCount != 16))
  { errorCode = "BAD_FIELD_COUNT"; return false; }
  if (moveMode == 1 && (!parseInt32(frame.fields[16], plan.endPanMdeg) ||
                        !parseInt32(frame.fields[17], plan.endTiltMdeg)))
  { errorCode = "VALUE_OUT_OF_RANGE"; return false; }
  plan.planVersion = planVersion;
  plan.moveMode = (uint8_t)moveMode;
  plan.disconnectPolicy = (DisconnectPolicyV2)disconnectPolicy;
  return true;
}

void V2Controller::sendCapabilities()
{
  _hardware.configureMotionLimits(_safety.panMinimumMdeg(), _safety.panMaximumMdeg(),
    _safety.tiltMinimumMdeg(), _safety.tiltMaximumMdeg());
  _protocol.beginFrame(V2_FLASH("CAPABILITIES"));
  _protocol.addSignedField(_safety.panMinimumMdeg());
  _protocol.addSignedField(_safety.panMaximumMdeg());
  _protocol.addSignedField(_safety.tiltMinimumMdeg());
  _protocol.addSignedField(_safety.tiltMaximumMdeg());
  _protocol.addField(_safety.limitsConfirmed() ? 1UL : 0UL);
  _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("TILT_CALIBRATION"));
  _protocol.addSignedField(_safety.tiltZeroOffsetMdeg());
  _protocol.addSignedField(TILT_ABSOLUTE_MIN_MDEG - _safety.tiltZeroOffsetMdeg());
  _protocol.addSignedField(TILT_ABSOLUTE_MAX_MDEG - _safety.tiltZeroOffsetMdeg());
  _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("NAVIGATION_CAPABILITIES"));
  _protocol.addField(1UL);
  _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("MOTION_CAPABILITIES"));
  _protocol.addField(1UL); // Profile tempo + optional per-axis manual tempo, no PWM scaling.
  _protocol.endFrame();
  _protocol.beginFrame("RUN_SPEED_CAPABILITIES"); _protocol.addField(2000UL); _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("MANUAL_STATUS_CAPABILITIES"));
  _protocol.addField(1UL);
  _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("MANUAL_JOG_CAPABILITIES"));
  _protocol.addField(_hardware.supportsManualJog() ? 1UL : 0UL); _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("MANUAL_JOG_VELOCITY_CAPABILITIES"));
  _protocol.addField((uint32_t)_hardware.manualJogMaximumVelocityMdeg()); _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("MANUAL_JOG_PAN_VELOCITY_CAPABILITIES"));
  _protocol.addField((uint32_t)_hardware.manualJogMaximumPanVelocityMdeg()); _protocol.endFrame();
  BenchDriveStatus bench;
  if(_hardware.getBenchStatus(bench))
  {
    _protocol.beginFrame(V2_FLASH("BENCH_CAPABILITIES"));
    _protocol.addField(3UL); _protocol.addField(40UL); _protocol.addField(3000UL);
    _protocol.addField(5000UL); _protocol.addField(20UL); _protocol.addField(192UL);
    _protocol.addField(5000UL); _protocol.addField(500UL); _protocol.addField(500UL); _protocol.endFrame();
  }
}

void V2Controller::sendDeviceInfo()
{
  _protocol.beginFrame(V2_FLASH("DEVICE_INFO"));
  _protocol.addField(V2_FLASH(PANODUINO_FIRMWARE_VERSION));
  _protocol.addField(V2_FLASH(PANODUINO_FIRMWARE_DATE));
  _protocol.addField((uint32_t)PANODUINO_PROTOCOL_VERSION);
  _protocol.addField(V2_FLASH(PANODUINO_HARDWARE_TARGET));
  _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("DEVICE_SESSION"));
  _protocol.addField(_safety.bootId());
  _protocol.endFrame();
}

void V2Controller::sendDiagnostics(uint32_t nowMs)
{
  MotionDiagnosticsV2 motion;
  _hardware.getMotionDiagnostics(motion);
  RunPhaseV2 phase = _executor.phase();
  uint32_t phaseElapsed = (uint32_t)(nowMs - _executor.phaseStartedMs());
  uint32_t movementElapsed = (phase == PHASE_MOVING || phase == PHASE_SETTLING)
    ? (uint32_t)(nowMs - _executor.movementStartedMs())
    : 0;
  _protocol.beginFrame(V2_FLASH("DIAGNOSTICS"));
  _protocol.addField(_executor.plan().runId);
  addRunStateField(_executor.state());
  addRunPhaseField(phase);
  _protocol.addField(_executor.nextImageIndex());
  _protocol.addField(motion.sampleSequence);
  _protocol.addField((uint32_t)motion.stableSamples);
  _protocol.addField((uint32_t)motion.requiredStableSamples);
  _protocol.addSignedField(motion.panTargetMdeg);
  _protocol.addSignedField(motion.tiltTargetMdeg);
  _protocol.addSignedField(_hardware.actualPanMdeg());
  _protocol.addSignedField(_hardware.actualTiltMdeg());
  _protocol.addSignedField(motion.panErrorMdeg);
  _protocol.addSignedField(motion.tiltErrorMdeg);
  _protocol.addField(motion.panWithinTolerance ? 1UL : 0UL);
  _protocol.addField(motion.tiltWithinTolerance ? 1UL : 0UL);
  _protocol.addField(phaseElapsed);
  _protocol.addField(movementElapsed);
  _protocol.addField(_executor.plan().settleTimeMs);
  _protocol.addField((uint32_t)PANORAMA_MOVEMENT_TIMEOUT_MS);
  _protocol.addField((uint32_t)_stackWatermark.currentFreeBytes());
  _protocol.addField((uint32_t)_stackWatermark.minimumFreeBytes());
  _protocol.endFrame();
  MotionTelemetryV2 telemetry;
  if (_hardware.getMotionTelemetry(telemetry))
  {
    _protocol.beginFrame(V2_FLASH("MOTION_DIAGNOSTICS"));
    _protocol.addSignedField(telemetry.panReferenceMdeg); _protocol.addSignedField(telemetry.tiltReferenceMdeg);
    _protocol.addSignedField(telemetry.panVelocityMdegPerSecond); _protocol.addSignedField(telemetry.tiltVelocityMdegPerSecond);
    _protocol.addSignedField(telemetry.panReferenceVelocity); _protocol.addSignedField(telemetry.tiltReferenceVelocity);
    _protocol.addSignedField(telemetry.panSignedPwm); _protocol.addSignedField(telemetry.tiltSignedPwm);
    _protocol.addField((uint32_t)telemetry.panPhase); _protocol.addField((uint32_t)telemetry.tiltPhase);
    _protocol.addField((uint32_t)telemetry.panDriveStage); _protocol.addField((uint32_t)telemetry.tiltDriveStage);
    _protocol.endFrame();
  }
}

#ifndef __AVR__
void V2Controller::sendDebugLog(uint16_t afterSequence)
{
  uint8_t startOffset = 0;
  bool found = afterSequence == 0;
  if (!found)
  {
    for (uint8_t offset = 0; offset < _debugLog.count(); offset++)
    {
      if (_debugLog.record(offset).sequence == afterSequence)
      {
        startOffset = (uint8_t)(offset + 1);
        found = true;
        break;
      }
    }
  }
  // Ist die Client-Sequenz bereits aus dem kleinen Ring gefallen, alle noch
  // vorhandenen Eintraege senden; dropped macht die Luecke sichtbar.
  if (!found) startOffset = 0;

  _pendingDebugCount = _pendingDebugOffset = 0;
  for (uint8_t offset = startOffset; offset < _debugLog.count(); offset++)
  {
    _pendingDebug[_pendingDebugCount++] = _debugLog.record(offset);
  }
  _pendingDebugNewest = _debugLog.newestSequence();
  _pendingDebugDropped = _debugLog.dropped();
  _pendingDebugActive = true;
  flushDebugLog();
}

void V2Controller::flushDebugLog()
{
  if (!_pendingDebugActive) return;
  // Leave room for command acknowledgements and status; snapshot survives ring changes.
  while (_pendingDebugOffset < _pendingDebugCount && _protocol.transmitSpace() >= 512)
    sendDebugRecord(_pendingDebug[_pendingDebugOffset++]);
  if (_pendingDebugOffset != _pendingDebugCount || _protocol.transmitSpace() < 512) return;
  _protocol.beginFrame(V2_FLASH("DEBUG_LOG_END"));
  _protocol.addField((uint32_t)_pendingDebugNewest);
  _protocol.addField((uint32_t)_pendingDebugCount);
  _protocol.addField((uint32_t)_pendingDebugDropped);
  _protocol.endFrame();
  _pendingDebugActive = false;
}

void V2Controller::sendDebugRecord(const DebugEventRecordV2 &record)
{
  int32_t panErrorMdeg = (int32_t)record.panErrorDeciDeg * 100L;
  int32_t tiltErrorMdeg = (int32_t)record.tiltErrorDeciDeg * 100L;
  int32_t actualPanMdeg = (int32_t)record.actualPanDeciDeg * 100L;
  int32_t actualTiltMdeg = (int32_t)record.actualTiltDeciDeg * 100L;
  int32_t targetPanMdeg = actualPanMdeg + panErrorMdeg;
  int32_t targetTiltMdeg = actualTiltMdeg + tiltErrorMdeg;
  _protocol.beginFrame(V2_FLASH("DEBUG_EVENT"));
  _protocol.addField((uint32_t)record.sequence);
  _protocol.addField(record.deviceTimeMs);
  _protocol.addField((uint32_t)record.code);
  _protocol.addField(_executor.plan().runId);
  _protocol.addField(record.imageIndex);
  addRunStateField((RunStateV2)record.state);
  addRunPhaseField((RunPhaseV2)record.phase);
  _protocol.addSignedField(targetPanMdeg);
  _protocol.addSignedField(targetTiltMdeg);
  _protocol.addSignedField(actualPanMdeg);
  _protocol.addSignedField(actualTiltMdeg);
  _protocol.addSignedField(panErrorMdeg);
  _protocol.addSignedField(tiltErrorMdeg);
  _protocol.addField((uint32_t)record.stableSamples);
  _protocol.endFrame();
}

void V2Controller::recordDebug(DebugEventCodeV2 code)
{
  MotionDiagnosticsV2 motion;
  _hardware.getMotionDiagnostics(motion);
  DebugEventRecordV2 record;
  record.sequence = 0;
  record.deviceTimeMs = _currentTickMs;
  uint32_t imageIndex = _executor.nextImageIndex();
  record.imageIndex = imageIndex > 65535UL ? 65535U : (uint16_t)imageIndex;
  int32_t actualPanMdeg = _hardware.actualPanMdeg();
  int32_t actualTiltMdeg = _hardware.actualTiltMdeg();
  long actualPanDeciDeg = actualPanMdeg / 100L;
  long actualTiltDeciDeg = actualTiltMdeg / 100L;
  if (actualPanDeciDeg > 32767L) actualPanDeciDeg = 32767L;
  if (actualPanDeciDeg < -32768L) actualPanDeciDeg = -32768L;
  if (actualTiltDeciDeg > 32767L) actualTiltDeciDeg = 32767L;
  if (actualTiltDeciDeg < -32768L) actualTiltDeciDeg = -32768L;
  record.actualPanDeciDeg = (int16_t)actualPanDeciDeg;
  record.actualTiltDeciDeg = (int16_t)actualTiltDeciDeg;
  long panDeciDeg = ((long)motion.panTargetMdeg - actualPanMdeg) / 100L;
  long tiltDeciDeg = ((long)motion.tiltTargetMdeg - actualTiltMdeg) / 100L;
  if (panDeciDeg > 32767L) panDeciDeg = 32767L;
  if (panDeciDeg < -32768L) panDeciDeg = -32768L;
  if (tiltDeciDeg > 32767L) tiltDeciDeg = 32767L;
  if (tiltDeciDeg < -32768L) tiltDeciDeg = -32768L;
  record.panErrorDeciDeg = (int16_t)panDeciDeg;
  record.tiltErrorDeciDeg = (int16_t)tiltDeciDeg;
  record.code = (uint8_t)code;
  record.state = (uint8_t)_executor.state();
  record.phase = (uint8_t)_executor.phase();
  record.stableSamples = motion.stableSamples;
  _debugLog.append(record);
}
#endif

void V2Controller::sendPlan()
{
  if (_executor.state() == RUN_IDLE) return;
  const PanoramaPlanV2 &plan = _executor.plan();
  _protocol.beginFrame(V2_FLASH("PLAN"));
  _protocol.addField(plan.runId);
  _protocol.addField((uint32_t)plan.planVersion);
  _protocol.addField((uint32_t)plan.moveMode);
  _protocol.addSignedField(plan.startPanMdeg);
  _protocol.addSignedField(plan.startTiltMdeg);
  _protocol.addField((uint32_t)plan.columns);
  _protocol.addField((uint32_t)plan.rows);
  _protocol.addSignedField(plan.stepPanMdeg);
  _protocol.addSignedField(plan.stepTiltMdeg);
  _protocol.addField(plan.settleTimeMs);
  _protocol.addField(plan.triggerPulseMs);
  _protocol.addField(plan.exposureWaitMs);
  _protocol.addField((uint32_t)plan.panSpeedPermille);
  _protocol.addField((uint32_t)plan.tiltSpeedPermille);
  _protocol.addField((uint32_t)plan.disconnectPolicy);
  _protocol.addField(plan.heartbeatTimeoutMs);
  if (plan.moveMode == 1)
  {
    _protocol.addSignedField(plan.endPanMdeg);
    _protocol.addSignedField(plan.endTiltMdeg);
  }
  _protocol.endFrame();
}

void V2Controller::sendSegments()
{
  for (uint8_t index = 0; index < _executor.segmentCount(); index++) sendSegment(index);
}

void V2Controller::sendSegment(uint8_t index)
{
  if (index >= _executor.segmentCount()) return;
  const RunSegmentV2 &segment = _executor.segment(index);
  _protocol.beginFrame(V2_FLASH("RUN_SEGMENT"));
  _protocol.addField(_executor.plan().runId);
  _protocol.addField((uint32_t)segment.revision);
  _protocol.addField(segment.startImageIndex);
  _protocol.addField(segment.captureCount);
  _protocol.endFrame();
}

void V2Controller::sendStatus()
{
  const TargetPositionV2 &target = _executor.currentTarget();
  const PanoramaPlanV2 &plan = _executor.plan();
  uint32_t total = (_executor.state() == RUN_IDLE) ? 0 : plan.totalImages();
  uint32_t currentSegmentCaptures = _executor.segmentCount() == 0
    ? 0
    : _executor.segment(_executor.segmentCount() - 1).captureCount;
  _protocol.beginFrame(V2_FLASH("STATUS"));
  _protocol.addField(_timelapseRunId != 0 ? _timelapseRunId : plan.runId);
  if (_timelapseActive) _protocol.addField(V2_FLASH("TIMELAPSE"));
  else if (_timelapseRunId != 0) _protocol.addField(V2_FLASH("TIMELAPSE_STOPPED"));
  else addRunStateField(_executor.state());
  addRunPhaseField(_executor.phase());
  _protocol.addField(_timelapseRunId != 0 ? _timelapseCaptureCount : _executor.completedImages());
  _protocol.addField(total);
  _protocol.addField((uint32_t)target.column);
  _protocol.addField((uint32_t)target.row);
  _protocol.addSignedField(target.panMdeg);
  _protocol.addSignedField(target.tiltMdeg);
  _protocol.addSignedField(_hardware.actualPanMdeg());
  _protocol.addSignedField(_hardware.actualTiltMdeg());
  _protocol.addField(_executor.pauseRequested() ? 1UL : 0UL);
  _protocol.addField(_eventSequence);
  _protocol.addField(_executor.lastError());
  _protocol.addField(_executor.nextImageIndex());
  _protocol.addField((uint32_t)_executor.planRevision());
  _protocol.addField((uint32_t)_executor.segmentCount());
  _protocol.addField(currentSegmentCaptures);
  _protocol.addField(_timelapseIntervalMs);
  _protocol.addField((uint32_t)_timelapsePulseMs);
  _protocol.endFrame();
  _protocol.beginFrame(V2_FLASH("STEP_TIMING"));
  _protocol.addField(plan.runId);
  _protocol.addField(target.imageIndex);
  _protocol.addField(_executor.activeMoveMaximumDurationMs());
  _protocol.endFrame();
  sendManualStatus();
}

void V2Controller::finishManualStatus(ManualResult result)
{
  if(_manualResult!=MANUAL_ACTIVE) return;
  if(_manualKind==MANUAL_JOG) { _jogActive=false; _lastStoppedJogToken=_jogToken; }
  _manualStatusDurationMs=(uint32_t)(_currentTickMs-_manualStatusStartedMs);
  _manualResult=result;
}

void V2Controller::sendManualStatus()
{
  // Retain the last result until another accepted action. GET_STATUS and the
  // periodic stream replay it independently of the bounded debug event ring.
  _protocol.beginFrame(V2_FLASH("MANUAL_STATUS"));
  _protocol.addField((uint32_t)_manualCommand);
  if(_manualKind==MANUAL_MOVE) _protocol.addField(V2_FLASH("MOVE"));
  else if(_manualKind==MANUAL_TRIGGER) _protocol.addField(V2_FLASH("TRIGGER"));
  else if(_manualKind==MANUAL_JOG) _protocol.addField(V2_FLASH("JOG"));
  else _protocol.addField(V2_FLASH("NONE"));
  switch(_manualResult)
  {
    case MANUAL_ACTIVE: _protocol.addField(V2_FLASH("ACTIVE")); break;
    case MANUAL_REACHED: _protocol.addField(V2_FLASH("REACHED")); break;
    case MANUAL_TRIGGERED: _protocol.addField(V2_FLASH("TRIGGERED")); break;
    case MANUAL_CANCELLED: _protocol.addField(V2_FLASH("CANCELLED")); break;
    case MANUAL_TIMED_OUT: _protocol.addField(V2_FLASH("TIMED_OUT")); break;
    case MANUAL_FAULT: _protocol.addField(V2_FLASH("FAULT")); break;
    case MANUAL_MISSED: _protocol.addField(V2_FLASH("TARGET_MISSED")); break;
    default: _protocol.addField(V2_FLASH("IDLE")); break;
  }
  _protocol.addSignedField(_manualStatusPan); _protocol.addSignedField(_manualStatusTilt);
  _protocol.addField(_manualResult==MANUAL_ACTIVE ? (uint32_t)(_currentTickMs-_manualStatusStartedMs) : _manualStatusDurationMs);
  _protocol.addField(_manualStatusMaximumMs);
  _protocol.endFrame();
}

void V2Controller::stopTimelapse()
{
  _timelapseActive = false;
  _timelapsePulseActive = false;
  _hardware.setCameraActive(false);
}

void V2Controller::tickTimelapse(uint32_t nowMs)
{
  if (!_timelapseActive) return;
  if (_timelapsePulseActive && (int32_t)(nowMs - _timelapsePulseEndMs) >= 0)
  {
    _hardware.setCameraActive(false);
    _timelapsePulseActive = false;
    _timelapseCaptureCount++;
    sendStatus();
  }
  if (!_timelapsePulseActive && (int32_t)(nowMs - _timelapseNextMs) >= 0)
  {
    _hardware.setCameraActive(true);
    _timelapsePulseActive = true;
    _timelapsePulseEndMs = nowMs + _timelapsePulseMs;
    _timelapseNextMs = nowMs + _timelapseIntervalMs;
  }
}

void V2Controller::checkHeartbeat(uint32_t nowMs)
{
  if(_benchActive && (uint32_t)(nowMs-_lastHeartbeatMs)>=1500)
  { emergencyStop("BENCH_CONNECTION_TIMEOUT"); sendStatus(); return; }
  if (!_sessionActive) return;
  const PanoramaPlanV2 &plan = _executor.plan();
  if (plan.disconnectPolicy != DISCONNECT_ESTOP_ON_TIMEOUT || plan.heartbeatTimeoutMs == 0) return;
  RunStateV2 state = _executor.state();
  if (!(state == RUN_RUNNING || state == RUN_PAUSE_PENDING || state == RUN_PAUSED)) return;
  if ((uint32_t)(nowMs - _lastHeartbeatMs) >= plan.heartbeatTimeoutMs)
  {
    _executor.emergencyStop("CONNECTION_TIMEOUT");
    sendStatus();
  }
}

void V2Controller::tickManualTrigger(uint32_t nowMs)
{
  if (_manualTriggerActive && (int32_t)(nowMs - _manualTriggerEndMs) >= 0)
  {
    _hardware.setCameraActive(false);
    _manualTriggerActive = false;
    finishManualStatus(MANUAL_TRIGGERED);
    sendManualImageTriggered(nowMs);
    sendStatus();
  }
}

void V2Controller::sendBenchStatus()
{
  BenchDriveStatus value;
  if(!_hardware.getBenchStatus(value)) return;
  _protocol.beginFrame(V2_FLASH("BENCH_STATUS"));
  _protocol.addField((uint32_t)value.count); _protocol.addField(value.active ? 1UL : 0UL);
  _protocol.addField(value.recording ? 1UL : 0UL); _protocol.addField(value.reason); _protocol.endFrame();
}

void V2Controller::tickBenchDrive(uint32_t)
{
  if(!_benchActive) return;
  BenchDriveStatus value;
  if(!_hardware.getBenchStatus(value)) { fault("BENCH_UNSUPPORTED"); return; }
  if(!value.active)
  {
    _hardware.stopAllMotion(); _safety.stopMotion();
    if(!value.recording) { _benchActive=false; sendBenchStatus(); sendStatus(); }
  }
}

void V2Controller::tickJog(uint32_t nowMs)
{
  if(!_jogActive) return;
  bool limit=_hardware.manualJogLimitReached();
  if(!_safety.motionEnabled() || limit || uint32_t(nowMs-_jogUpdatedMs)>=_hardware.manualJogLeaseMs())
  {
    _hardware.stopAllMotion(); _safety.stopMotion();
    finishManualStatus(limit ? MANUAL_REACHED : MANUAL_CANCELLED); sendStatus();
  }
}

void V2Controller::tickManualMove(uint32_t nowMs)
{
  if (!_manualMoveActive) return;
  if (_hardware.targetsReached())
  {
    // A partial inward recovery may intentionally finish outside a limit.
    // Do not hold/re-fault there and block the user's next inward step.
    if (_safety.validateTarget(_hardware.actualPanMdeg(), _hardware.actualTiltMdeg()) != SAFETY_OK)
    { _hardware.stopAllMotion(); _safety.stopMotion(); }
    _manualMoveActive = false;
    finishManualStatus(MANUAL_REACHED);
    recordDebug(DEBUG_MANUAL_MOVE_REACHED);
    sendStatus();
  }
  else if ((uint32_t)(nowMs - _manualMoveStartedMs) >= _manualMoveMaximumMs)
  {
    _manualMoveActive = false;
    finishManualStatus(MANUAL_TIMED_OUT);
    if(_manualIncremental) _executor.emergencyStop("STEP_MOVE_TIMEOUT");
    else _executor.fault("MANUAL_MOVE_TIMEOUT");
    sendStatus();
  }
}

void V2Controller::sendManualImageTriggered(uint32_t nowMs)
{
  _manualCaptureSequence++;
  _eventSequence++;
  ManualCaptureRecordV2 record;
  record.eventSequence = _eventSequence;
  record.captureSequence = _manualCaptureSequence;
  uint32_t rasterCaptures = _executor.completedImages();
  record.rasterCaptureCount = rasterCaptures > 65535UL ? 65535U : (uint16_t)rasterCaptures;
  record.panMdeg = _hardware.actualPanMdeg();
  record.tiltMdeg = _hardware.actualTiltMdeg();
  record.deviceTimeMs = nowMs;
  record.attachedToRun = _executor.state() == RUN_PAUSED;

  recordDebug(DEBUG_MANUAL_TRIGGERED);
  sendManualRecord(record, false);
}

void V2Controller::sendManualRecord(const ManualCaptureRecordV2 &record, bool replayed)
{
  _protocol.beginFrame(V2_FLASH("MANUAL_IMAGE_TRIGGERED"));
  _protocol.addField(record.eventSequence);
  _protocol.addField(record.attachedToRun ? _executor.plan().runId : 0UL);
  _protocol.addField(record.captureSequence);
  _protocol.addField(record.rasterCaptureCount);
  _protocol.addSignedField(record.panMdeg);
  _protocol.addSignedField(record.tiltMdeg);
  _protocol.addField(record.deviceTimeMs);
  _protocol.addField(replayed ? 1UL : 0UL);
  _protocol.endFrame();
}

void V2Controller::addRunStateField(RunStateV2 state)
{
  switch (state)
  {
    case RUN_IDLE: _protocol.addField(V2_FLASH("IDLE")); break;
    case RUN_PLAN_READY: _protocol.addField(V2_FLASH("PLAN_READY")); break;
    case RUN_RUNNING: _protocol.addField(V2_FLASH("RUNNING")); break;
    case RUN_PAUSE_PENDING: _protocol.addField(V2_FLASH("PAUSE_PENDING")); break;
    case RUN_PAUSED: _protocol.addField(V2_FLASH("PAUSED")); break;
    case RUN_STOPPED: _protocol.addField(V2_FLASH("STOPPED")); break;
    case RUN_COMPLETED: _protocol.addField(V2_FLASH("COMPLETED")); break;
    default: _protocol.addField(V2_FLASH("FAULT")); break;
  }
}

void V2Controller::addRunPhaseField(RunPhaseV2 phase)
{
  switch (phase)
  {
    case PHASE_SELECT_TARGET: _protocol.addField(V2_FLASH("SELECT_TARGET")); break;
    case PHASE_MOVING: _protocol.addField(V2_FLASH("MOVING")); break;
    case PHASE_SETTLING: _protocol.addField(V2_FLASH("SETTLING")); break;
    case PHASE_TRIGGER_ACTIVE: _protocol.addField(V2_FLASH("TRIGGER_ACTIVE")); break;
    case PHASE_EXPOSURE_WAIT: _protocol.addField(V2_FLASH("EXPOSURE_WAIT")); break;
    case PHASE_COMMIT_IMAGE: _protocol.addField(V2_FLASH("COMMIT_IMAGE")); break;
    default: _protocol.addField(V2_FLASH("NONE")); break;
  }
}

bool V2Controller::parseUInt32(const char *value, uint32_t &result) const
{
  if (value == 0 || value[0] == 0 || value[0] == '-') return false;
  uint32_t parsed = 0;
  while (*value != 0)
  {
    if (*value < '0' || *value > '9') return false;
    uint8_t digit = (uint8_t)(*value++ - '0');
    if (parsed > (UINT32_MAX - digit) / 10UL) return false;
    parsed = parsed * 10UL + digit;
  }
  result = parsed;
  return true;
}

bool V2Controller::parseUInt16(const char *value, uint16_t &result) const
{
  uint32_t parsed;
  if (!parseUInt32(value, parsed) || parsed > 65535UL) return false;
  result = (uint16_t)parsed;
  return true;
}

bool V2Controller::parseInt32(const char *value, int32_t &result) const
{
  if (value == 0 || value[0] == 0) return false;
  bool negative = *value == '-';
  if (negative) value++;
  if (*value == 0) return false;
  uint32_t magnitude = 0;
  uint32_t maximum = negative ? 2147483648UL : 2147483647UL;
  while (*value != 0)
  {
    if (*value < '0' || *value > '9') return false;
    uint8_t digit = (uint8_t)(*value++ - '0');
    if (magnitude > (maximum - digit) / 10UL) return false;
    magnitude = magnitude * 10UL + digit;
  }
  if (negative)
    result = magnitude == 2147483648UL ? INT32_MIN : -(int32_t)magnitude;
  else
    result = (int32_t)magnitude;
  return true;
}

bool V2Controller::requireRunId(const V2Frame &frame, uint32_t &runId, ProtocolV2 &protocol)
{
  if (frame.fieldCount != 1 || !parseUInt32(frame.fields[0], runId))
  {
    protocol.sendNack(frame.sequence, V2_FLASH("BAD_FIELD_COUNT"));
    return false;
  }
  return true;
}


bool V2Controller::configurationWritable() const
{
  return !_safety.motionEnabled() && !_manualMoveActive && !_manualTriggerActive &&
    !_timelapseActive && !_benchActive && _executor.plan().runId==0 &&
    (_executor.state()==RUN_IDLE || _executor.state()==RUN_STOPPED);
}

void V2Controller::sendMotionConfigurationInfo()
{
  MotionSettings settings; uint32_t revision=0;
  if(!_hardware.getMotionConfiguration(settings,revision)) return;
  _protocol.beginFrame("MOTION_CONFIG_INFO");
  _protocol.addField((uint32_t)MotionSettings::SCHEMA); _protocol.addField(revision);
  _protocol.addField((uint32_t)MOTION_PARAMETER_COUNT);
  _protocol.addField(revision ? "STORED" : "DEFAULTS"); _protocol.addField(_configToken);
  _protocol.endFrame();
  _protocol.beginFrame("CAPTURE_WARNING_SETTINGS");
  _protocol.addSignedField(settings.values[MP_CAPTURE_PAN_WARNING_DEG]);
  _protocol.addSignedField(settings.values[MP_CAPTURE_TILT_WARNING_DEG]);
  _protocol.addField(revision); _protocol.endFrame();
}

bool V2Controller::handleMotionConfiguration(const V2Frame &f,ProtocolV2 &p,uint32_t now)
{
  bool get=V2_MESSAGE_IS(f.messageType,"GET_MOTION_CONFIG"),
    begin=V2_MESSAGE_IS(f.messageType,"BEGIN_MOTION_CONFIG"),
    set=V2_MESSAGE_IS(f.messageType,"SET_MOTION_PARAMETER"),
    apply=V2_MESSAGE_IS(f.messageType,"APPLY_MOTION_CONFIG"),
    cancel=V2_MESSAGE_IS(f.messageType,"CANCEL_MOTION_CONFIG");
  bool warning=V2_MESSAGE_IS(f.messageType,"SET_CAPTURE_WARNING_TOLERANCE");
  if(!get && !begin && !set && !apply && !cancel && !warning) return false;
  MotionSettings active; uint32_t revision=0;
  if(!_hardware.getMotionConfiguration(active,revision))
  { p.sendNack(f.sequence,"CONFIG_UNSUPPORTED"); return true; }
  if(get)
  {
    uint16_t index=0;
    if(f.fieldCount!=1 || !parseUInt16(f.fields[0],index) || index>=MOTION_PARAMETER_COUNT)
    { p.sendNack(f.sequence,"BAD_PARAMETER_INDEX"); return true; }
    const MotionParameterDescriptor &d=motionParameter(index);
    p.sendAck(f.sequence,"OK");
    p.beginFrame("MOTION_CONFIG"); p.addField((uint32_t)MotionSettings::SCHEMA); p.addField(revision);
    p.addField((uint32_t)MOTION_PARAMETER_COUNT); p.addField((uint32_t)index); p.addField(d.name);
    p.addSignedField(active.values[index]); p.addField((uint32_t)d.scale); p.addField(d.unit);
    p.addSignedField(d.minimum); p.addSignedField(d.maximum); p.addSignedField(d.defaultValue);
    p.endFrame(); return true;
  }
  if(!configurationWritable()) { p.sendNack(f.sequence,"CONFIG_BUSY"); return true; }
  if(warning)
  {
    int32_t pan=0,tilt=0; uint32_t base=0;
    if(_configToken) { p.sendNack(f.sequence,"CONFIG_EDIT_ACTIVE"); return true; }
    if(f.fieldCount!=3 || !parseInt32(f.fields[0],pan) || !parseInt32(f.fields[1],tilt) ||
       !parseUInt32(f.fields[2],base) || base!=revision || base==UINT32_MAX)
    { p.sendNack(f.sequence,"CONFIG_REVISION"); return true; }
    if(!active.set(MP_CAPTURE_PAN_WARNING_DEG,pan) || !active.set(MP_CAPTURE_TILT_WARNING_DEG,tilt))
    { p.sendNack(f.sequence,"CONFIG_VALUE_RANGE"); return true; }
    // Repeating the desired values does not wear EEPROM or create a revision.
    MotionSettings current; uint32_t ignored=0; _hardware.getMotionConfiguration(current,ignored);
    const char *error=0;
    if(current.values[MP_CAPTURE_PAN_WARNING_DEG]!=pan || current.values[MP_CAPTURE_TILT_WARNING_DEG]!=tilt)
      error=_hardware.applyMotionConfiguration(active,base+1);
    if(error) p.sendNack(f.sequence,error);
    else { p.sendAck(f.sequence,"WARNING_TOLERANCE_SAVED"); sendMotionConfigurationInfo(); }
    return true;
  }
  uint32_t token=0;
  if(!f.fieldCount || !parseUInt32(f.fields[0],token) || !token)
  { p.sendNack(f.sequence,"BAD_CONFIG_TOKEN"); return true; }
  if(begin)
  {
    uint16_t schema=0; uint32_t base=0;
    if(f.fieldCount!=3 || !parseUInt16(f.fields[1],schema) || schema!=MotionSettings::SCHEMA ||
       !parseUInt32(f.fields[2],base) || base!=revision)
    { p.sendNack(f.sequence,"CONFIG_REVISION"); return true; }
    if(_configToken)
    {
      if(_configToken!=token || _configRevision!=base) p.sendNack(f.sequence,"CONFIG_EDIT_ACTIVE");
      else p.sendAck(f.sequence,"CONFIG_BEGUN");
      return true; // Identical begin never discards already staged values.
    }
    _stagedMotion=active; _configRevision=base; _configToken=token;
    _configStartedMs=_configUpdatedMs=now; memset(_configSeen,0,sizeof(_configSeen));
    p.sendAck(f.sequence,"CONFIG_BEGUN"); return true;
  }
  if(apply && f.fieldCount==2)
  {
    uint32_t base=0;
    if(parseUInt32(f.fields[1],base) && token==_lastAppliedConfigToken &&
       base==_lastAppliedConfigBase && revision==base+1)
    { p.sendAck(f.sequence,"CONFIG_APPLIED"); sendMotionConfigurationInfo(); return true; }
  }
  if(token!=_configToken) { p.sendNack(f.sequence,"CONFIG_TOKEN"); return true; }
  if(cancel)
  {
    if(f.fieldCount!=1) p.sendNack(f.sequence,"BAD_FIELD_COUNT");
    else { _configToken=0; p.sendAck(f.sequence,"CONFIG_CANCELLED"); }
    return true;
  }
  if(set)
  {
    uint16_t index=0; int32_t value=0;
    if(f.fieldCount!=3 || !parseUInt16(f.fields[1],index) || !parseInt32(f.fields[2],value) ||
       !_stagedMotion.set(index,value)) { p.sendNack(f.sequence,"CONFIG_VALUE_RANGE"); return true; }
    _configSeen[index/8]|=(uint8_t)(1U<<(index%8)); _configUpdatedMs=now;
    p.sendAck(f.sequence,"CONFIG_STAGED"); return true;
  }
  uint32_t base=0;
  if(f.fieldCount!=2 || !parseUInt32(f.fields[1],base) || base!=_configRevision || revision!=base || base==UINT32_MAX)
  { p.sendNack(f.sequence,"CONFIG_REVISION"); return true; }
  for(unsigned int i=0;i<MOTION_PARAMETER_COUNT;++i)
    if(!(_configSeen[i/8]&(1U<<(i%8)))) { p.sendNack(f.sequence,"CONFIG_INCOMPLETE"); return true; }
  const char *error=_stagedMotion.validate();
  if(error) { p.sendNack(f.sequence,error); return true; }
  error=_hardware.applyMotionConfiguration(_stagedMotion,base+1);
  if(error) { p.sendNack(f.sequence,error); return true; }
  _lastAppliedConfigToken=token; _lastAppliedConfigBase=base; _configToken=0;
  p.sendAck(f.sequence,"CONFIG_APPLIED"); sendMotionConfigurationInfo(); return true;
}
