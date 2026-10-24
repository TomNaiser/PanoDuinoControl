#include "PanoramaExecutor.h"
#include <string.h>

PanoramaExecutor::PanoramaExecutor(IPanoramaHardware &hardware, SafetyController &safety, IExecutorEvents &events)
  : _hardware(hardware), _safety(safety), _events(events), _state(RUN_IDLE), _phase(PHASE_NONE),
    _phaseStartedMs(0), _movementStartedMs(0), _incrementalMoveBudgetMs(0), _stepTimedOut(false), _passiveSettling(false), _passiveWaitStartedMs(0), _captureReady(false), _capturePanMdeg(0), _captureTiltMdeg(0), _nextImageIndex(0), _retriedImageIndex(UINT32_MAX), _captureCount(0), _planRevision(0),
    _segmentCount(0), _pauseRequested(false), _pendingRewindColumns(0),
    _pendingForwardColumns(0), _lastNavigationForward(false),
    _pendingSettleMs(0), _pendingTriggerMs(0), _pendingExposureMs(0),
    _timingPending(false), _timingRevision(0)
{
  memset(&_target, 0, sizeof(_target));
  strcpy(_lastError, "NONE");
}

const char *PanoramaExecutor::loadPlan(const PanoramaPlanV2 &plan)
{
  if (!(_state == RUN_IDLE || _state == RUN_PLAN_READY)) return "INVALID_STATE";
  const char *validation = plan.validate(_safety);
  if (strcmp(validation, "OK") != 0) return validation;
  _plan = plan;
  _nextImageIndex = 0;
  _retriedImageIndex = UINT32_MAX;
  _captureCount = 0;
  _planRevision = 0;
  _segmentCount = 1;
  _segments[0].revision = 0;
  _segments[0].startImageIndex = 0;
  _segments[0].captureCount = 0;
  _pauseRequested = false;
  _pendingRewindColumns = 0;
  _pendingForwardColumns = 0;
  _lastNavigationForward = false;
  _timingPending = false;
  _timingRevision = 0;
  strcpy(_lastError, "NONE");
  transition(RUN_PLAN_READY, PHASE_NONE);
  return "PLAN_ACCEPTED";
}

const char *PanoramaExecutor::start(uint32_t runId, uint32_t nowMs)
{
  if (_state != RUN_PLAN_READY) return "INVALID_STATE";
  if (_plan.runId != runId) return "RUN_ID_MISMATCH";
  _phaseStartedMs = nowMs;
  transition(RUN_RUNNING, PHASE_SELECT_TARGET);
  return "STARTED";
}

const char *PanoramaExecutor::pause(uint32_t runId)
{
  if (_plan.runId != runId) return "RUN_ID_MISMATCH";
  if (_state != RUN_RUNNING && _state != RUN_PAUSE_PENDING) return "INVALID_STATE";
  _pauseRequested = true;
  if (_phase == PHASE_SELECT_TARGET)
  {
    transition(RUN_PAUSED, PHASE_NONE);
    return "PAUSED";
  }
  transition(RUN_PAUSE_PENDING, _phase);
  return "PAUSE_PENDING";
}

const char *PanoramaExecutor::resume(uint32_t runId)
{
  if (_plan.runId != runId) return "RUN_ID_MISMATCH";
  if (_state != RUN_PAUSED) return "INVALID_STATE";
  _pauseRequested = false;
  strcpy(_lastError,"NONE");
  transition(RUN_RUNNING, PHASE_SELECT_TARGET);
  return "RESUMED";
}

const char *PanoramaExecutor::rewindColumns(uint32_t runId, uint16_t columns)
{
  if (_plan.moveMode != 0) return "UNSUPPORTED_MODE";
  if (_plan.runId != runId) return "RUN_ID_MISMATCH";
  if (!(_state == RUN_PAUSED || _state == RUN_RUNNING || _state == RUN_PAUSE_PENDING)) return "INVALID_STATE";
  if (columns == 0) return "VALUE_OUT_OF_RANGE";
  if (_segmentCount >= MAX_RUN_SEGMENTS_V2) return "SEGMENT_LIMIT";
  if (_pendingForwardColumns != 0) return "NAVIGATION_PENDING";
  uint32_t requested = (uint32_t)_pendingRewindColumns + columns;
  uint16_t combined = (uint16_t)(requested > _plan.columns ? _plan.columns : requested);
  if (_state == RUN_PAUSED)
  {
    applyRewindColumns(combined);
    return "REWOUND";
  }
  _pendingRewindColumns = combined;
  return "REWIND_PENDING";
}

const char *PanoramaExecutor::forwardColumns(uint32_t runId, uint16_t columns)
{
  if (_plan.moveMode != 0) return "UNSUPPORTED_MODE";
  if (_plan.runId != runId) return "RUN_ID_MISMATCH";
  if (!(_state == RUN_PAUSED || _state == RUN_RUNNING || _state == RUN_PAUSE_PENDING)) return "INVALID_STATE";
  if (columns == 0) return "VALUE_OUT_OF_RANGE";
  if (_segmentCount >= MAX_RUN_SEGMENTS_V2) return "SEGMENT_LIMIT";
  if (_pendingRewindColumns != 0) return "NAVIGATION_PENDING";
  uint32_t currentColumn = (_state == RUN_PAUSED || _phase == PHASE_SELECT_TARGET)
    ? _nextImageIndex / _plan.rows : _target.column;
  uint32_t requested = (uint32_t)_pendingForwardColumns + columns;
  if (currentColumn + requested >= _plan.columns) return "LAST_COLUMN";
  if (_state == RUN_PAUSED)
  {
    applyForwardColumns((uint16_t)requested);
    return "FORWARDED";
  }
  _pendingForwardColumns = (uint16_t)requested;
  return "FORWARD_PENDING";
}

const char *PanoramaExecutor::retryColumn(uint32_t runId,uint16_t previous)
{
  if(_plan.runId!=runId) return "RUN_ID_MISMATCH";
  if(_state!=RUN_PAUSED) return "INVALID_STATE";
  if(_plan.moveMode!=0) return "UNSUPPORTED_MODE";
  if(previous>1 || _nextImageIndex>=_plan.totalImages()) return "VALUE_OUT_OF_RANGE";
  // An empty segment can be reused: repeated failed starts consume no slots.
  if(_segments[_segmentCount-1].captureCount==0) --_segmentCount;
  else if(_segmentCount>=MAX_RUN_SEGMENTS_V2) return "SEGMENT_LIMIT";
  applyRewindColumns(previous);
  strcpy(_lastError,"NONE");
  return "COLUMN_RETRY_READY"; // Deliberately does not resume or enable motors.
}

void PanoramaExecutor::applyForwardColumns(uint16_t columns)
{
  uint32_t currentColumn = (_state == RUN_PAUSED || _phase == PHASE_SELECT_TARGET)
    ? _nextImageIndex / _plan.rows : _target.column;
  _nextImageIndex = (currentColumn + columns) * _plan.rows;
  _planRevision++;
  RunSegmentV2 &newSegment = _segments[_segmentCount++];
  newSegment.revision = _planRevision;
  newSegment.startImageIndex = (uint16_t)_nextImageIndex;
  newSegment.captureCount = 0;
  _pendingForwardColumns = 0;
  _lastNavigationForward = true;
  memset(&_target, 0, sizeof(_target));
}

void PanoramaExecutor::applyRewindColumns(uint16_t columns)
{
  uint32_t currentColumn = _nextImageIndex >= _plan.totalImages()
    ? (uint32_t)(_plan.columns - 1)
    : _nextImageIndex / _plan.rows;
  uint32_t targetColumn = currentColumn > columns ? currentColumn - columns : 0;
  _nextImageIndex = targetColumn * _plan.rows;
  _planRevision++;
  RunSegmentV2 &newSegment = _segments[_segmentCount++];
  newSegment.revision = _planRevision;
  newSegment.startImageIndex = (uint16_t)_nextImageIndex; // Planvalidierung begrenzt auf 4000 Bilder.
  newSegment.captureCount = 0;
  _pendingRewindColumns = 0;
  _lastNavigationForward = false;
  memset(&_target, 0, sizeof(_target));
}

const char *PanoramaExecutor::updateTiming(uint32_t runId, uint32_t settleMs, uint32_t triggerMs, uint32_t exposureMs)
{
  if (_plan.runId != runId) return "RUN_ID_MISMATCH";
  if (!(_state == RUN_PLAN_READY || _state == RUN_RUNNING || _state == RUN_PAUSE_PENDING || _state == RUN_PAUSED))
    return "INVALID_STATE";
  if (!PanoramaPlanV2::timingValid(settleMs, triggerMs, exposureMs)) return "VALUE_OUT_OF_RANGE";
  if (_state == RUN_PLAN_READY || _state == RUN_PAUSED ||
      (_state == RUN_RUNNING && _phase == PHASE_SELECT_TARGET))
  {
    applyTiming(settleMs, triggerMs, exposureMs);
    return "TIMING_UPDATED";
  }
  _pendingSettleMs = settleMs;
  _pendingTriggerMs = triggerMs;
  _pendingExposureMs = exposureMs;
  _timingPending = true;
  return "TIMING_QUEUED";
}

void PanoramaExecutor::applyTiming(uint32_t settleMs, uint32_t triggerMs, uint32_t exposureMs)
{
  _plan.settleTimeMs = settleMs;
  _plan.triggerPulseMs = triggerMs;
  _plan.exposureWaitMs = exposureMs;
  _timingPending = false;
  _timingRevision++;
}

const char *PanoramaExecutor::stop(uint32_t runId, const char *reason)
{
  if (_plan.runId != runId && _state != RUN_IDLE) return "RUN_ID_MISMATCH";
  if (!(_state == RUN_RUNNING || _state == RUN_PAUSE_PENDING || _state == RUN_PAUSED || _state == RUN_PLAN_READY ||
        _state == RUN_IDLE || _state == RUN_COMPLETED || _state == RUN_STOPPED)) return "INVALID_STATE";
  _hardware.stopAllMotion();
  _hardware.setCameraActive(false);
  _safety.stopMotion();
  _pendingRewindColumns = 0;
  _pendingForwardColumns = 0;
  _timingPending = false;
  strncpy(_lastError, reason == 0 ? "USER_STOP" : reason, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = 0;
  transition(RUN_STOPPED, PHASE_NONE);
  return "STOPPED";
}

void PanoramaExecutor::emergencyStop(const char *reason)
{
  _hardware.stopAllMotion();
  _hardware.setCameraActive(false);
  _safety.stopMotion();
  _pendingRewindColumns = 0;
  _pendingForwardColumns = 0;
  _timingPending = false;
  strncpy(_lastError, reason == 0 ? "ESTOP" : reason, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = 0;
  transition(RUN_STOPPED, PHASE_NONE);
}

void PanoramaExecutor::fault(const char *reason)
{
  _hardware.stopAllMotion();
  _hardware.setCameraActive(false);
  _safety.stopMotion();
  _pendingRewindColumns = 0;
  _pendingForwardColumns = 0;
  _timingPending = false;
  strncpy(_lastError, reason == 0 ? "FAULT" : reason, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = 0;
  transition(RUN_FAULT, PHASE_NONE);
}

bool PanoramaExecutor::stopMissedTarget(const char *reason,uint32_t nowMs)
{
  // Position accuracy is a quality warning for every panorama target,
  // including the initial position and column return. Keep unrelated faults.
  if(!reason || strcmp(reason,"MOTION_TARGET_MISSED") || _passiveSettling ||
     !(_state==RUN_RUNNING || _state==RUN_PAUSE_PENDING) ||
     !(_phase==PHASE_MOVING || _phase==PHASE_SETTLING) ||
     _safety.validateTarget(_hardware.actualPanMdeg(),_hardware.actualTiltMdeg())!=SAFETY_OK) return false;
  _hardware.stopAllMotion(); _hardware.setCameraActive(false); _safety.stopMotion();
  _stepTimedOut=true; beginPassiveSettling(nowMs);
  strcpy(_lastError,"STEP_TARGET_MISSED"); transition(_state,PHASE_SETTLING); return true;
}

bool PanoramaExecutor::pauseRecoverable(const char *reason)
{
  if(!reason || !(_state==RUN_RUNNING || _state==RUN_PAUSE_PENDING) ||
     _safety.validateTarget(_hardware.actualPanMdeg(),_hardware.actualTiltMdeg())!=SAFETY_OK) return false;
  if(strcmp(reason,"EXPOSURE_UNSTABLE") && strcmp(reason,"CAPTURE_NOT_STABLE") &&
     strcmp(reason,"MOTION_STALLED") && strcmp(reason,"MOTION_SETTLE_TIMEOUT") &&
     strcmp(reason,"MOTION_FINE_TIMEOUT") && strcmp(reason,"MOVEMENT_TIMEOUT") &&
     strcmp(reason,"MOTION_NOT_QUIET")) return false;
  _hardware.stopAllMotion(); _hardware.setCameraActive(false); _safety.stopMotion();
  strncpy(_lastError,reason,sizeof(_lastError)-1);_lastError[sizeof(_lastError)-1]=0;
  // One retry at the same uncommitted image. A requested pause/navigation
  // wins, and RESUME/reconnect cannot renew the budget at this image.
  if(strcmp(reason,"EXPOSURE_UNSTABLE") && strcmp(reason,"CAPTURE_NOT_STABLE") &&
     (_phase==PHASE_MOVING || _phase==PHASE_SETTLING) &&
     _state==RUN_RUNNING && !_pauseRequested && !_pendingRewindColumns && !_pendingForwardColumns &&
     _retriedImageIndex!=_nextImageIndex)
  {
    _retriedImageIndex=_nextImageIndex;
    _events.onAcquisitionRetry(_nextImageIndex,reason);
    transition(RUN_RUNNING,PHASE_SELECT_TARGET);
    return true;
  }
  _pauseRequested=false; _pendingRewindColumns=_pendingForwardColumns=0;
  transition(RUN_PAUSED,PHASE_NONE);
  return true;
}

void PanoramaExecutor::beginPassiveSettling(uint32_t nowMs)
{
  _passiveSettling=true;_passiveWaitStartedMs=_phaseStartedMs=nowMs;
  _capturePanMdeg=_hardware.actualPanMdeg();_captureTiltMdeg=_hardware.actualTiltMdeg();
  _captureReady=_hardware.exposurePositionStable(_capturePanMdeg,_captureTiltMdeg);
}

void PanoramaExecutor::clearRun()
{
  if (!(_state == RUN_STOPPED || _state == RUN_COMPLETED || _state == RUN_FAULT)) return;
  _plan.runId = 0;
  _nextImageIndex = 0;
  _captureCount = 0;
  _planRevision = 0;
  _segmentCount = 0;
  _pauseRequested = false;
  _pendingRewindColumns = 0;
  _pendingForwardColumns = 0;
  _timingPending = false;
  _timingRevision = 0;
  strcpy(_lastError, "NONE");
  transition(RUN_IDLE, PHASE_NONE);
}

void PanoramaExecutor::tick(uint32_t nowMs)
{
  if (!(_state == RUN_RUNNING || _state == RUN_PAUSE_PENDING)) return;
  if(!_passiveSettling && (_phase==PHASE_MOVING || (_phase==PHASE_SETTLING && !_hardware.targetsReached())) && _incrementalMoveBudgetMs &&
     uint32_t(nowMs-_movementStartedMs)>=_incrementalMoveBudgetMs)
  {
    _hardware.stopAllMotion(); _hardware.setCameraActive(false); _safety.stopMotion();
    _stepTimedOut=true; beginPassiveSettling(nowMs); strcpy(_lastError,"STEP_MOVE_TIMEOUT");
    transition(_state,PHASE_SETTLING); return;
  }
  // After a deadline stop, this guard bounds passive settling, not motor drive.
  if (!_passiveSettling && (_phase == PHASE_MOVING || _phase == PHASE_SETTLING) &&
      uint32_t(nowMs-_movementStartedMs) >= PANORAMA_MOVEMENT_TIMEOUT_MS +
        (_stepTimedOut ? _incrementalMoveBudgetMs : 0))
  { if(!pauseRecoverable("MOVEMENT_TIMEOUT")) fault("MOVEMENT_TIMEOUT"); return; }
  if ((_phase == PHASE_TRIGGER_ACTIVE || _phase == PHASE_EXPOSURE_WAIT || _phase==PHASE_COMMIT_IMAGE) &&
      !_hardware.exposurePositionStable(_capturePanMdeg,_captureTiltMdeg))
  { if(!pauseRecoverable("EXPOSURE_UNSTABLE")) fault("EXPOSURE_UNSTABLE"); return; }

  switch (_phase)
  {
    case PHASE_SELECT_TARGET:
      if (_pauseRequested)
      {
        _hardware.stopAllMotion();
        _safety.stopMotion();
        transition(RUN_PAUSED, PHASE_NONE);
        return;
      }
      if (_timingPending) applyTiming(_pendingSettleMs, _pendingTriggerMs, _pendingExposureMs);
      if (_pendingRewindColumns != 0) applyRewindColumns(_pendingRewindColumns);
      if (_pendingForwardColumns != 0) applyForwardColumns(_pendingForwardColumns);
      if (_nextImageIndex >= _plan.totalImages())
      {
        transition(RUN_COMPLETED, PHASE_NONE);
        return;
      }
      if (!selectCurrentTarget())
      {
        fault("TARGET_CALCULATION");
        return;
      }
      {
      // Only the grid's return to row zero of a later column uses the
      // configured return tempo. Initial positioning, row steps, spherical
      // bands and the Pan axis retain their requested plan speeds.
      uint16_t tiltTempo=_plan.tiltSpeedPermille;
      bool columnReturn=_plan.moveMode==0 && _plan.rows>1 && _target.row==0 && _target.imageIndex>0;
      if(columnReturn)
        tiltTempo=_hardware.columnReturnTiltTempo(tiltTempo);
      bool positioning=_target.imageIndex==0 || (_plan.moveMode==0 && _plan.rows>1 && _target.row==0) ||
        (_plan.moveMode==1 && _target.column==0);
      bool accepted=columnReturn ? _hardware.setColumnReturnTargetMdeg(_target.panMdeg,_target.tiltMdeg,_plan.panSpeedPermille,tiltTempo) :
        positioning ? _hardware.setPositioningTargetMdeg(_target.panMdeg,_target.tiltMdeg,_plan.panSpeedPermille,tiltTempo) :
        _hardware.setTargetMdeg(_target.panMdeg, _target.tiltMdeg, _plan.panSpeedPermille, tiltTempo, false);
      if (!accepted)
      {
        fault("TARGET_REJECTED");
        return;
      }
      }
      _stepTimedOut=false; _passiveSettling=false; strcpy(_lastError,"NONE");
      _safety.enableMotion();
      _phaseStartedMs = nowMs;
      _movementStartedMs = nowMs;
      _incrementalMoveBudgetMs=_target.imageIndex>0 &&
        !(_plan.moveMode==0 && _plan.rows>1 && _target.row==0) &&
        !(_plan.moveMode==1 && _target.column==0)
        ? _hardware.incrementalMoveMaximumDurationMs() : _hardware.positioningMoveMaximumDurationMs();
      transition(_state, PHASE_MOVING);
      return;

    case PHASE_MOVING:
      {
        uint32_t quietSince;
        bool candidate = _hardware.targetQuietSince(quietSince);
        if (candidate || _hardware.targetsReached())
        {
          _phaseStartedMs = candidate ? quietSince : nowMs;
          if(_hardware.targetsReached())
          {
            _hardware.stopAllMotion(); _safety.stopMotion(); beginPassiveSettling(nowMs);
          }
          transition(_state, PHASE_SETTLING);
        }
      }
      return;

    case PHASE_SETTLING:
      if(!_passiveSettling && _hardware.targetsReached())
      {
        _hardware.stopAllMotion(); _safety.stopMotion(); beginPassiveSettling(nowMs);
      }
      if(_passiveSettling)
      {
        // Confirmed arrival and deadline stops both coast passively. Never
        // re-enable torque while settling or restart strict arrival counting.
        if(uint32_t(nowMs-_passiveWaitStartedMs)>=_plan.settleTimeMs+_hardware.captureStabilityWaitMs())
        { if(!pauseRecoverable("CAPTURE_NOT_STABLE")) fault("CAPTURE_NOT_STABLE"); return; }
        int32_t pan=_hardware.actualPanMdeg(),tilt=_hardware.actualTiltMdeg();
        bool stable=(_stepTimedOut ? _hardware.stoppedPositionResting() :
            _hardware.stoppedPositionStable(_capturePanMdeg,_captureTiltMdeg)) &&
            _hardware.exposurePositionStable(_capturePanMdeg,_captureTiltMdeg) &&
            _hardware.exposurePositionStable(pan,tilt);
        if(!stable) {
          _captureReady=false;_phaseStartedMs=nowMs;
          _capturePanMdeg=pan;_captureTiltMdeg=tilt;
        } else {
          if(!_captureReady) {_captureReady=true;_phaseStartedMs=nowMs;}
          if(elapsed(nowMs,_plan.settleTimeMs))
          {
            _capturePanMdeg=pan;_captureTiltMdeg=tilt;
            _hardware.setCameraActive(true); _phaseStartedMs=nowMs;
            transition(_state,PHASE_TRIGGER_ACTIVE);
          }
        }
        return;
      }
      // Before confirmed arrival, a lost strict candidate returns to MOVING.
      // Afterwards the passive branch above owns the separate settle clock.
      {
        uint32_t quietSince;
        if(_hardware.targetQuietSince(quietSince)) _phaseStartedMs=quietSince;
        else transition(_state,PHASE_MOVING);
      }
      return;

    case PHASE_TRIGGER_ACTIVE:
      if (elapsed(nowMs, _plan.triggerPulseMs))
      {
        _hardware.setCameraActive(false);
        _phaseStartedMs = nowMs;
        transition(_state, PHASE_EXPOSURE_WAIT);
      }
      return;

    case PHASE_EXPOSURE_WAIT:
      if (elapsed(nowMs, _plan.exposureWaitMs))
      {
        transition(_state, PHASE_COMMIT_IMAGE);
      }
      return;

    case PHASE_COMMIT_IMAGE:
      _captureCount++;
      _segments[_segmentCount - 1].captureCount++;
      _events.onImageCompleted(_target, _hardware.actualPanMdeg(), _hardware.actualTiltMdeg(), _captureCount, _planRevision);
      _nextImageIndex++;
      if (_timingPending) applyTiming(_pendingSettleMs, _pendingTriggerMs, _pendingExposureMs);
      if (_pendingRewindColumns != 0) applyRewindColumns(_pendingRewindColumns);
      if (_pendingForwardColumns != 0) applyForwardColumns(_pendingForwardColumns);
      if (_pauseRequested)
      {
        _hardware.stopAllMotion();
        _safety.stopMotion();
        transition(RUN_PAUSED, PHASE_NONE);
      }
      else if (_nextImageIndex >= _plan.totalImages())
      {
        transition(RUN_COMPLETED, PHASE_NONE);
      }
      else
      {
        transition(RUN_RUNNING, PHASE_SELECT_TARGET);
      }
      return;

    default:
      fault("INVALID_PHASE");
      return;
  }
}

void PanoramaExecutor::transition(RunStateV2 state, RunPhaseV2 phase)
{
  bool stateChanged = _state != state;
  RunPhaseV2 previousPhase = _phase;
  _state = state;
  _phase = phase;
  if (stateChanged) _events.onStateChanged(state);
  if (previousPhase != phase) _events.onPhaseChanged(previousPhase, phase);
}

bool PanoramaExecutor::selectCurrentTarget()
{
  return _plan.targetFor(_nextImageIndex, _target) && _safety.validateTarget(_target.panMdeg, _target.tiltMdeg) == SAFETY_OK;
}

bool PanoramaExecutor::elapsed(uint32_t nowMs, uint32_t durationMs) const
{
  return (uint32_t)(nowMs - _phaseStartedMs) >= durationMs;
}
