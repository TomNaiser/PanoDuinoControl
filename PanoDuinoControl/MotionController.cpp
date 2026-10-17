#include "MotionController.h"
#include "PanStopModel.h"
#include <math.h>
#include <string.h>

static double bounded(double value, double minimum, double maximum)
{ return value < minimum ? minimum : value > maximum ? maximum : value; }
static int8_t polarity(double value) { return value > 0 ? 1 : value < 0 ? -1 : 0; }
static double bezier(const double *points, unsigned int count, double time)
{
  double work[6];
  for (unsigned int i = 0; i < count; ++i) work[i] = points[i];
  for (unsigned int level = count - 1; level > 0; --level)
    for (unsigned int i = 0; i < level; ++i) work[i] += time * (work[i + 1] - work[i]);
  return work[0];
}

uint32_t MotionController::plannedMoveMaximumDurationMs() const
{
  uint32_t floor=_settings.milliseconds(MP_INCREMENTAL_MOVE_MAX_DURATION_MS);
  if(!_settings.values[MP_MOVE_AUTO_DURATION]) return floor;
  double seconds=0;
  for(unsigned int i=0;i<2;++i)
  {
    const Axis &axis=_axes[i];
    if(!axis.active) continue;
    double duration=axis.profile.duration();
    if(axis.positioningTarget || axis.manualTarget)
    {
      // These targets use the velocity ramp, rather than the Bezier clock.
      double distance=fabs(axis.target-meanPosition(axis));
      double acceleration=axis.positioningAcceleration>0 ? axis.positioningAcceleration :
        _settings.number(i==0 ? MP_PAN_JOG_ACCELERATION : MP_JOG_ACCELERATION);
      double speed=axis.jogMaximumSpeed;
      if(speed>0 && acceleration>0)
      {
        duration=distance<=speed*speed/acceleration ? 2*sqrt(distance/acceleration) :
          distance/speed+speed/acceleration;
        duration+=fabs(axis.velocity)/acceleration;
      }
    }
    seconds=fmax(seconds,duration); // Both axes travel simultaneously.
  }
  double budget=ceil(seconds*1000*_settings.number(MP_MOVE_DURATION_FACTOR))+
    _settings.milliseconds(MP_MOVE_DURATION_RESERVE_MS)+_settings.milliseconds(MP_CONFIRM_MS);
  uint32_t ceiling=_settings.milliseconds(MP_MOVE_AUTO_MAX_MS);
  if(!isfinite(budget) || budget>=ceiling) return ceiling;
  return budget<=floor ? floor : (uint32_t)budget;
}

MotionProfile::MotionProfile() : _duration(0) { memset(_points, 0, sizeof(_points)); }

bool MotionProfile::plan(double position, double velocity, double acceleration, double target,
                        double minimum, double maximum, uint16_t speedPermille,
                        const AxisMotionConfiguration &configuration)
{
  if (!isfinite(position) || !isfinite(velocity) || !isfinite(acceleration) || !isfinite(target) ||
      !isfinite(minimum) || !isfinite(maximum) || minimum >= maximum ||
      speedPermille == 0 || speedPermille > 1000 ||
      configuration.maximumVelocity <= 0 || configuration.maximumAcceleration <= 0 || configuration.maximumJerk <= 0)
    return false;
  bool targetAllowed = position < minimum ? target >= position && target <= maximum :
    position > maximum ? target <= position && target >= minimum : target >= minimum && target <= maximum;
  if (!targetAllowed) return false;
  double vMax = configuration.maximumVelocity * speedPermille / 1000.0;
  // A moving replan may need time to reach a lower requested speed.
  double velocityBound = fmax(vMax, fabs(velocity));
  double aMax = configuration.maximumAcceleration, jMax = configuration.maximumJerk;
  if (fabs(acceleration) > aMax + 1e-6) return false;
  double distance = fabs(target - position);
  bool resting = fabs(velocity) < 1e-8 && fabs(acceleration) < 1e-8;
  double duration = fmax(0.10, fmax(1.875 * distance / vMax,
    fmax(sqrt(5.774 * distance / aMax), pow(60.0 * distance / jMax, 1.0 / 3.0))));
  // Outside a limit only the inward recovery interval is admitted.
  double allowedMinimum = fmin(minimum, position), allowedMaximum = fmax(maximum, position);
  for (unsigned int attempt = 0; attempt < 32; ++attempt)
  {
    if (duration > PANORAMA_MOVEMENT_TIMEOUT_MS / 1000.0 - 2.0) return false;
    _points[0] = position;
    _points[1] = position + velocity * duration / 5.0;
    _points[2] = position + 2.0 * velocity * duration / 5.0 + acceleration * duration * duration / 20.0;
    _points[3] = _points[4] = _points[5] = target;
    double scale = 1.0;
    for (unsigned int i = 0; i < 6; ++i)
      if (_points[i] < allowedMinimum - 1e-8 || _points[i] > allowedMaximum + 1e-8) return false;
    if (!resting)
    {
      // Convex-hull bounds certify the whole profile, not just sampled points.
      for (unsigned int i = 0; i < 5; ++i)
        scale = fmax(scale, fabs(5.0 * (_points[i + 1] - _points[i]) / duration) / velocityBound);
      for (unsigned int i = 0; i < 4; ++i)
        scale = fmax(scale, sqrt(fabs(20.0 * (_points[i + 2] - 2 * _points[i + 1] + _points[i]) /
          (duration * duration)) / aMax));
      for (unsigned int i = 0; i < 3; ++i)
        scale = fmax(scale, pow(fabs(60.0 * (_points[i + 3] - 3 * _points[i + 2] +
          3 * _points[i + 1] - _points[i]) / (duration * duration * duration)) / jMax, 1.0 / 3.0));
    }
    if (scale <= 1.000001) { _duration = duration; return true; }
    duration *= fmax(1.05, scale * 1.02);
  }
  return false;
}

void MotionProfile::evaluate(double seconds, double &position, double &velocity, double &acceleration) const
{
  if (_duration == 0 || seconds >= _duration)
  { position = _points[5]; velocity = acceleration = 0; return; }
  double time = bounded(seconds / _duration, 0, 1);
  double first[5], second[4];
  for (unsigned int i = 0; i < 5; ++i) first[i] = 5 * (_points[i + 1] - _points[i]) / _duration;
  for (unsigned int i = 0; i < 4; ++i) second[i] = 4 * (first[i + 1] - first[i]) / _duration;
  position = bezier(_points, 6, time);
  velocity = bezier(first, 5, time);
  acceleration = bezier(second, 4, time);
}

MotionController::MotionController() : _lastSampleMs(0), _sampleSequence(0), _hasSample(false), _fault(0)
{
  // Global construction runs with the R4 boot stack limited to 1024 bytes.
  // Use our already-constructed settings; never create full config temporaries.
  initializeAxes(_settings.axis(false),_settings.axis(true));
}

MotionController::MotionController(const AxisMotionConfiguration &panConfiguration,
  const AxisMotionConfiguration &tiltConfiguration) : _lastSampleMs(0), _sampleSequence(0), _hasSample(false), _fault(0)
{
  _settings.setAxis(false,panConfiguration); _settings.setAxis(true,tiltConfiguration);
  initializeAxes(panConfiguration,tiltConfiguration);
}

void MotionController::initializeAxes(const AxisMotionConfiguration &panConfiguration,
  const AxisMotionConfiguration &tiltConfiguration)
{
  // Axis contains a constructed MotionProfile; initialize the remaining data explicitly.
  for (unsigned int i = 0; i < 2; ++i)
  {
    Axis &axis = _axes[i];
    axis.configuration = i == 0 ? panConfiguration : tiltConfiguration;
    axis.minimum = -180; axis.maximum = 180;
    axis.position = axis.velocity = axis.target = axis.reference = axis.referenceVelocity = axis.referenceAcceleration = 0;
    axis.profileTime = axis.output = axis.integral = axis.reversalPosition = axis.progressPosition = 0;
    axis.travelStartPosition = axis.travelSpeed = axis.jogMaximumSpeed = axis.positioningAcceleration = 0; axis.jog=axis.jogVelocity=axis.manualTarget=axis.positioningTarget=false;
    axis.fineVelocity=axis.fineAcceleration=axis.fineStartPosition=0;
    axis.fineStartedMs=0; axis.fineUsed=false; axis.brakeQuiet=false; axis.brakeQuietStartedMs=0;
    axis.driveStageStartedMs = axis.startReliefStartedMs = 0; axis.startReliefUsed=false; axis.travelDirection = 0; axis.driveStage = DRIVE_IDLE;
    memset(axis.history, 0, sizeof(axis.history));
    memset(axis.historyTimes, 0, sizeof(axis.historyTimes));
    axis.quietStartedMs = axis.reversalStartedMs = axis.progressStartedMs = 0;
    axis.historyCount = axis.historyIndex = 0;
    axis.driveDirection = axis.reversalDirection = axis.integralDirection = 0;
    axis.active = axis.profileComplete = axis.correcting = axis.reversing = axis.quiet = axis.progressWatch = axis.reversalWatch = false;
  }
}

void MotionController::configureLimits(double panMinimum, double panMaximum, double tiltMinimum, double tiltMaximum)
{
  _axes[0].minimum = panMinimum; _axes[0].maximum = panMaximum;
  _axes[1].minimum = tiltMinimum; _axes[1].maximum = tiltMaximum;
}

bool MotionController::configure(const MotionSettings &settings)
{
  if(settings.validate() || !configurationAllowed()) return false;
  _settings=settings;
  _axes[0].configuration=settings.axis(false); _axes[1].configuration=settings.axis(true);
  return true;
}

bool MotionController::command(double panTarget, double tiltTarget, uint16_t panSpeed, uint16_t tiltSpeed)
{
  if (!_hasSample || _fault || !panSpeed || !tiltSpeed || panSpeed>2000 || tiltSpeed>2000) return false;
  double targets[2] = {panTarget, tiltTarget};
  uint16_t speeds[2] = {panSpeed, tiltSpeed};
  MotionProfile plans[2];
  bool changed[2];
  for (unsigned int i = 0; i < 2; ++i)
  {
    Axis &axis = _axes[i];
    changed[i] = !axis.active || fabs(targets[i] - axis.target) > 1e-8;
    if (!changed[i]) continue; // Preserve the other axis's holding state and estimator.
    double start = axis.active && !axis.profileComplete ? axis.reference : axis.position;
    bool following = axis.active && !axis.profileComplete;
    // Starting from OFF/HOLD uses the same windowed estimate as arrival.
    // Differentiating a single noisy encoder increment can invent an opposing
    // velocity larger than the requested profile speed and reject a safe step.
    double velocity = following ? axis.referenceVelocity : quietVelocity(axis, _lastSampleMs);
    // An OFF/HOLD start tolerates the same bounded drift as an established hold;
    // strict arrival confirmation remains independent and unchanged.
    if (!following && span(axis) <= _settings.number(MP_POSITION_EXIT_DEG) && fabs(velocity) <= _settings.number(MP_VELOCITY_EXIT_DPS))
      velocity = 0;
    double acceleration = axis.active && !axis.profileComplete ? axis.referenceAcceleration : 0;
    if(stagedTravel(axis))
    {
      // Its old nominal Bezier curve does not describe the physical staged
      // move. Validate the new target from actual feedback; controlTravel
      // separately coasts an opposing velocity before driving the new way.
      start=meanPosition(axis); velocity=acceleration=0;
    }
    AxisMotionConfiguration profileConfiguration=axis.configuration;
    uint16_t profileTempo=speeds[i];
    if(profileTempo>1000) { profileConfiguration.maximumVelocity=targetVelocity(i==1,profileTempo); profileTempo=1000; }
    if (!plans[i].plan(start, velocity, acceleration, targets[i], axis.minimum, axis.maximum, profileTempo, profileConfiguration))
      return false; // Neither axis changes on rejection.
  }
  for (unsigned int i = 0; i < 2; ++i)
  {
    Axis &axis = _axes[i];
    if(axis.manualTarget || axis.positioningTarget) { axis.correcting=true; axis.quiet=false; }
    axis.manualTarget=false;
    axis.positioningTarget=false;
    axis.positioningAcceleration=0;
    if (!changed[i]) continue;
    if(!axis.active) { axis.integral=0; axis.integralDirection=0; }
    axis.jog=axis.jogVelocity=axis.manualTarget=false;
    axis.profile = plans[i]; axis.target = targets[i]; axis.profileTime = 0;
    axis.profile.evaluate(0, axis.reference, axis.referenceVelocity, axis.referenceAcceleration);
    axis.active = axis.correcting = true;
    axis.profileComplete = axis.reversing = axis.quiet = axis.progressWatch = axis.reversalWatch = false;
    axis.progressPosition = axis.position;
    if(stagedTravel(axis))
    {
      axis.travelStartPosition=meanPosition(axis);
      axis.travelDirection=polarity(axis.target-axis.travelStartPosition);
      axis.travelSpeed=targetVelocity(i==1,speeds[i]);
      axis.driveStage=DRIVE_START; axis.driveStageStartedMs=_lastSampleMs; axis.startReliefUsed=false; axis.startReliefStartedMs=0;
      axis.integral=0;
      axis.fineUsed=false; axis.fineStartedMs=0; axis.brakeQuiet=false; axis.brakeQuietStartedMs=0;
    }
  }
  return true;
}

bool MotionController::commandManual(double pan,double tilt,uint16_t panSpeed,uint16_t tiltSpeed)
{
  if(!command(pan,tilt,panSpeed,tiltSpeed)) return false;
  _axes[0].manualTarget=_axes[1].manualTarget=true;
  Axis &tiltAxis=_axes[1];
  if(stagedTravel(tiltAxis)) {
    tiltAxis.positioningTarget=true;
    tiltAxis.jogMaximumSpeed=targetVelocity(true,tiltSpeed);
    tiltAxis.travelSpeed=0; tiltAxis.reference=meanPosition(tiltAxis);
    tiltAxis.referenceVelocity=tiltAxis.referenceAcceleration=0;
    if(feedbackResting(_lastSampleMs)) tiltAxis.driveDirection=tiltAxis.travelDirection;
  }
  Axis &axis=_axes[0];
  if(!stagedTravel(axis)) {
    // Finite Plan targets use the proven Pan velocity drive, with their own
    // endpoint, rather than waiting for a weak, slowly advancing PD profile.
    axis.travelStartPosition=meanPosition(axis);
    axis.travelDirection=polarity(axis.target-axis.travelStartPosition);
    axis.jogMaximumSpeed=targetVelocity(false,panSpeed);
    axis.travelSpeed=0; axis.reference=axis.travelStartPosition;
    axis.referenceVelocity=axis.referenceAcceleration=0;
    axis.profileComplete=false; axis.output=axis.integral=0;
    axis.driveStage=DRIVE_START; axis.driveStageStartedMs=_lastSampleMs;
    // A confirmed rest is an ordinary start, even after the previous move
    // drove the other way. Moving reversals keep the original torque guard.
    if(feedbackResting(_lastSampleMs)) axis.driveDirection=axis.travelDirection;
  }
  return true;
}

bool MotionController::commandJog(bool tiltAxis,int direction,uint16_t tempo)
{
  if((direction!=1 && direction!=-1) || tempo<1 || tempo>1000 || !feedbackResting(_lastSampleMs)) return false;
  Axis &axis=_axes[tiltAxis ? 1 : 0];
  double margin=_settings.number(MP_JOG_LIMIT_MARGIN);
  double target=direction>0 ? axis.maximum-margin : axis.minimum+margin;
  if((target-axis.position)*direction<=0) return false;
  double pan=tiltAxis ? _axes[0].position : target;
  double tilt=tiltAxis ? target : _axes[1].position;
  // Use the full-tempo profile only to validate limits; jog owns the slowly
  // advancing reference and never queues or repeatedly replans small steps.
  if(!command(pan,tilt,1000,1000)) return false;
  Axis &other=_axes[tiltAxis ? 0 : 1]; other.active=false; other.output=other.integral=0; other.jog=other.jogVelocity=false;
  axis.jog=true; axis.jogVelocity=false; axis.jogMaximumSpeed=axis.configuration.maximumVelocity*
    fmin(tempo,_settings.values[MP_JOG_MAX_TEMPO])/1000.0;
  axis.reference=meanPosition(axis); axis.referenceVelocity=axis.referenceAcceleration=0;
  // Rest was confirmed above: this is a start, not an unconfirmed moving
  // reversal. Keep the slew limit, but allow tracking torque to build.
  axis.travelDirection=(int8_t)direction; axis.travelSpeed=0; axis.output=axis.integral=0; axis.driveDirection=(int8_t)direction;
  axis.travelStartPosition=meanPosition(axis); axis.driveStage=DRIVE_START; axis.driveStageStartedMs=_lastSampleMs;
  return true;
}

bool MotionController::commandPositioning(double pan,double tilt,uint16_t panSpeed,uint16_t tiltSpeed)
{
  if(!commandManual(pan,tilt,panSpeed,tiltSpeed)) return false;
  for(unsigned int i=0;i<2;++i) { _axes[i].manualTarget=false; _axes[i].positioningTarget=true; }
  return true;
}

double MotionController::targetVelocity(bool tilt,uint16_t tempo) const
{
  double speed=_axes[tilt?1:0].configuration.maximumVelocity*tempo/1000.0;
  return tempo<=1000 ? speed : fmin(speed,_settings.number(tilt ? MP_RUN_TILT_MAX_VELOCITY : MP_RUN_PAN_MAX_VELOCITY));
}

bool MotionController::commandColumnReturn(double pan,double tilt,uint16_t panSpeed,uint16_t tiltSpeed)
{
  if(!commandPositioning(pan,tilt,panSpeed,tiltSpeed)) return false;
  Axis &axis=_axes[1];
  if(stagedTravel(axis) && axis.travelDirection>0 && _settings.values[MP_COLUMN_RETURN_VELOCITY]>0) {
    axis.jogMaximumSpeed=_settings.number(MP_COLUMN_RETURN_VELOCITY);
    axis.positioningAcceleration=_settings.number(MP_COLUMN_RETURN_ACCELERATION);
  }
  return true;
}

bool MotionController::commandJogVelocity(bool tiltAxis,int direction,double velocityDps)
{
  if(!isfinite(velocityDps) || velocityDps<0.5 || velocityDps>_settings.number(tiltAxis ? MP_JOG_MAX_VELOCITY : MP_PAN_JOG_MAX_VELOCITY)) return false;
  if(!commandJog(tiltAxis,direction,1000)) return false;
  Axis &axis=_axes[tiltAxis ? 1 : 0]; axis.jogVelocity=true; axis.jogMaximumSpeed=velocityDps;
  return true;
}

void MotionController::observe(Axis &axis, double position, double dt, bool resetVelocity, uint32_t nowMs)
{
  axis.position = position;
  if (resetVelocity) axis.historyCount = axis.historyIndex = 0;
  axis.history[axis.historyIndex] = position;
  axis.historyTimes[axis.historyIndex] = nowMs;
  axis.historyIndex = (uint8_t)((axis.historyIndex + 1) % MOTION_HISTORY_LENGTH);
  if (axis.historyCount < MOTION_HISTORY_LENGTH) ++axis.historyCount;
  // Estimate speed across several timestamped encoder samples before filtering.
  // A single 20 ms difference amplifies quantization into large brake pulses.
  double alpha = dt / (_settings.number(MP_FEEDBACK_FILTER_SECONDS) + dt);
  double measured = windowVelocity(axis, nowMs, MOTION_FEEDBACK_HISTORY_LENGTH);
  axis.velocity = resetVelocity ? 0 : axis.velocity + alpha * (measured - axis.velocity);
}

double MotionController::span(const Axis &axis)
{
  if (axis.historyCount < MOTION_HISTORY_LENGTH) return 1e6;
  double minimum = axis.history[0], maximum = axis.history[0];
  for (unsigned int i = 1; i < axis.historyCount; ++i)
  { minimum = fmin(minimum, axis.history[i]); maximum = fmax(maximum, axis.history[i]); }
  return maximum - minimum;
}

double MotionController::quietVelocity(const Axis &axis, uint32_t nowMs)
{
  if (axis.historyCount < MOTION_HISTORY_LENGTH) return axis.velocity;
  return windowVelocity(axis, nowMs, MOTION_HISTORY_LENGTH);
}

double MotionController::meanPosition(const Axis &axis)
{ return recentPosition(axis,MOTION_HISTORY_LENGTH); }

double MotionController::recentPosition(const Axis &axis, uint8_t maximumSamples)
{
  if(!axis.historyCount) return axis.position;
  uint8_t count=axis.historyCount<maximumSamples ? axis.historyCount : maximumSamples;
  double sum=0;
  for(uint8_t i=0;i<count;++i)
    sum+=axis.history[(axis.historyIndex+MOTION_HISTORY_LENGTH-1-i)%MOTION_HISTORY_LENGTH];
  return sum/count;
}

double MotionController::windowVelocity(const Axis &axis, uint32_t nowMs, uint8_t maximumSamples)
{
  unsigned int count = axis.historyCount < maximumSamples ? axis.historyCount : maximumSamples;
  if (count < 2) return axis.velocity;
  double sumTime = 0, sumPosition = 0;
  for (unsigned int i = 0; i < count; ++i)
  {
    unsigned int index = (axis.historyIndex + MOTION_HISTORY_LENGTH - 1 - i) % MOTION_HISTORY_LENGTH;
    sumTime -= (uint32_t)(nowMs - axis.historyTimes[index]) / 1000.0;
    sumPosition += axis.history[index];
  }
  double timeMean = sumTime / count, positionMean = sumPosition / count;
  double covariance = 0, variance = 0;
  for (unsigned int i = 0; i < count; ++i)
  {
    unsigned int index = (axis.historyIndex + MOTION_HISTORY_LENGTH - 1 - i) % MOTION_HISTORY_LENGTH;
    double time = -(double)(uint32_t)(nowMs - axis.historyTimes[index]) / 1000.0 - timeMean;
    covariance += time * (axis.history[index] - positionMean); variance += time * time;
  }
  return variance > 1e-12 ? covariance / variance : axis.velocity;
}

bool MotionController::stagedTravel(const Axis &axis)
{ return axis.configuration.positiveRunningPwm>0 && axis.configuration.negativeRunningPwm>0; }

double MotionController::jogVelocityGain(const Axis &axis,MotionParameterId low,MotionParameterId high) const
{
  double fraction=bounded(axis.travelSpeed/_settings.number(&axis==&_axes[0] ? MP_PAN_JOG_MAX_VELOCITY : MP_JOG_MAX_VELOCITY),0,1);
  return _settings.number(low)+fraction*(_settings.number(high)-_settings.number(low));
}

double MotionController::controlTravel(Axis &axis, uint32_t nowMs, double dt)
{
  double position=meanPosition(axis), velocity=quietVelocity(axis,nowMs);
  double remaining=(axis.target-recentPosition(axis,3))*axis.travelDirection;
  if(axis.driveStage==DRIVE_START && fabs(axis.target-position)<=_settings.number(MP_TRAVEL_POSITION_ENTER_DEG) &&
     fabs(velocity)<=_settings.number(MP_VELOCITY_ENTER_DPS) && span(axis)<=_settings.number(MP_QUIET_SPAN_DEG))
  {
    axis.driveStage=DRIVE_SETTLE; axis.profileComplete=true; axis.correcting=false;
    axis.reference=axis.target; axis.referenceVelocity=axis.referenceAcceleration=0; axis.output=axis.integral=0;
    return 0;
  }
  if(axis.driveStage==DRIVE_FINE) return controlFine(axis,nowMs,dt);
  if(axis.driveStage==DRIVE_SETTLE)
  {
    if(fabs(axis.target-position)>_settings.number(MP_TRAVEL_POSITION_EXIT_DEG))
    { fail("MOTION_TARGET_MISSED"); return 0; }
    return 0;
  }
  if(axis.driveStage==DRIVE_BRAKE)
  {
    // Braking is committed: do not reapply breakaway torque or chase noise.
    // Output has already been removed; independently establish actual rest.
    if(fabs(axis.output)<0.5 && fabs(velocity)<=_settings.number(MP_VELOCITY_ENTER_DPS) &&
       span(axis)<=_settings.number(MP_QUIET_SPAN_DEG))
    {
      // Require a full stationary confirmation interval before evaluating
      // residual error. History lag at first rest must not trigger a restart.
      if(!axis.brakeQuiet) { axis.brakeQuiet=true; axis.brakeQuietStartedMs=nowMs; }
      if(uint32_t(nowMs-axis.brakeQuietStartedMs)<_settings.milliseconds(MP_CONFIRM_MS)) return 0;
      if(fabs(axis.target-position)-_settings.number(MP_TRAVEL_POSITION_ENTER_DEG)>1e-9)
      {
        if(_settings.values[MP_FINE_ENABLED] && _settings.values[MP_FINE_MAX_PASSES]==1 && !axis.fineUsed &&
           (axis.target-position)*axis.travelDirection>0 &&
           fabs(axis.target-position)<=_settings.number(MP_FINE_MAX_DISTANCE))
        { beginFine(axis,nowMs,0); return 0; }
        fail("MOTION_TARGET_MISSED"); return 0;
      }
      axis.driveStage=DRIVE_SETTLE; axis.correcting=false;
    }
    else { axis.brakeQuiet=false;
      if(uint32_t(nowMs-axis.driveStageStartedMs)>_settings.milliseconds(MP_TRAVEL_SETTLE_TIMEOUT_MS))
      { fail("MOTION_NOT_QUIET"); return 0; }
    }
    return 0;
  }
  // History-centre lag and the observed low-speed coast. The faster velocity
  // estimate protects short startup moves before the quiet window catches up.
  // This lead is provisional, not an identified mechanical stopping model.
  uint8_t brakeSamples=(axis.jogVelocity || axis.manualTarget || axis.positioningTarget) ?
    (uint8_t)_settings.values[MP_JOG_VELOCITY_SAMPLES] : 3;
  double speed=fmax(axis.velocity*axis.travelDirection,windowVelocity(axis,nowMs,brakeSamples)*axis.travelDirection);
  bool starting=axis.driveStage==DRIVE_START || uint32_t(nowMs-axis.driveStageStartedMs)<_settings.milliseconds(MP_TRAVEL_BRAKE_START_WINDOW_MS);
  double lead=starting ? _settings.number(MP_TRAVEL_START_BRAKE_LEAD_SECONDS) : _settings.number(MP_TRAVEL_BRAKE_LEAD_SECONDS);
  if(axis.travelDirection<0) lead=fmax(lead,_settings.number(MP_TRAVEL_DOWN_BRAKE_LEAD_SECONDS));
  if(axis.jogVelocity && !axis.positioningTarget) lead=fmax(lead,_settings.number(MP_JOG_VELOCITY_BRAKE_LEAD));
  double stoppingDistance=fmax(0.0,speed)*lead;
  if((axis.manualTarget || axis.positioningTarget) && axis.travelDirection<0) {
    // Both estimates describe the same passive coast. Adding them (and
    // switching the second off after startup) alternates early stops and
    // overshoots. Use the larger estimate consistently through the move.
    double coastSpeed=fmax(0.0,speed);
    stoppingDistance=fmax(stoppingDistance,coastSpeed*coastSpeed/(2*_settings.number(MP_MANUAL_TILT_DOWN_COAST_DECELERATION)));
  }
  if(remaining<=stoppingDistance)
  {
    axis.driveStage=DRIVE_BRAKE; axis.driveStageStartedMs=nowMs; axis.brakeQuiet=false;
    axis.integral=axis.output=0; axis.profileComplete=true;
    axis.reference=axis.target; axis.referenceVelocity=axis.referenceAcceleration=0;
    axis.reversing=axis.reversalWatch=axis.progressWatch=false;
    return 0;
  }
  if(axis.driveStage==DRIVE_START)
  {
    bool continuousStart=axis.jogVelocity || axis.manualTarget || axis.positioningTarget;
    if(uint32_t(nowMs-axis.driveStageStartedMs)>=_settings.milliseconds(MP_TRAVEL_START_TIMEOUT_MS))
    { fail("MOTION_STALLED"); return 0; }
    // An opposite moving replan first coasts to rest; never force a reversal
    // merely because the requested target changed while the mount was moving.
    if(!axis.jog && velocity*axis.travelDirection < -_settings.number(MP_VELOCITY_EXIT_DPS)) return 0;
    double startProgress=recentPosition(axis,continuousStart ? (uint8_t)_settings.values[MP_JOG_VELOCITY_SAMPLES] : 3)-axis.travelStartPosition;
    double startVelocity=continuousStart ? windowVelocity(axis,nowMs,(uint8_t)_settings.values[MP_JOG_VELOCITY_SAMPLES]) : axis.velocity;
    if(startProgress*axis.travelDirection>=_settings.number(continuousStart ? MP_JOG_VELOCITY_START_PROGRESS : MP_TRAVEL_START_PROGRESS_DEG) &&
       startVelocity*axis.travelDirection>_settings.number(MP_VELOCITY_EXIT_DPS))
    {
      axis.driveStage=DRIVE_CRUISE; axis.driveStageStartedMs=nowMs;
      // Preload the lower running torque only when short-window feedback
      // already approaches the requested speed; weak progress keeps the bias.
      axis.integral=windowVelocity(axis,nowMs,3)*axis.travelDirection>=_settings.number(MP_TRAVEL_START_PRELOAD_SPEED_FRACTION)*axis.travelSpeed ? -_settings.number(MP_TRAVEL_PWM_REDUCTION_MAX) : 0;
      // Remove excess breakaway torque immediately while retaining running
      // torque. A long decay from 40 causes large overshoot in short moves.
      double floor=axis.travelDirection>0 ? axis.configuration.positiveRunningPwm : axis.configuration.negativeRunningPwm;
      if(continuousStart) axis.integral=fabs(axis.output)-floor-
        (axis.jogVelocity ? jogVelocityGain(axis,MP_TRAVEL_LOW_P_GAIN,MP_JOG_VELOCITY_P_GAIN) : _settings.number(MP_TRAVEL_LOW_P_GAIN))*(axis.travelSpeed-startVelocity*axis.travelDirection);
      else axis.output=axis.travelDirection*fmin(fabs(axis.output),floor);
    }
    else
    {
      double bias=axis.travelDirection>0 ? axis.configuration.positiveRunningPwm : axis.configuration.negativeRunningPwm;
      double running=fmax(1.0,bias-_settings.number(MP_TRAVEL_PWM_REDUCTION_MAX));
      // Early progress may precede the confirmed three-sample transition.
      // Relieve torque once, without declaring movement/arrival confirmed.
      // If it was noise, resume the bounded start after 100 ms; do not retry
      // the relief or extend the independent 1.5-second start watchdog.
      if(!continuousStart && !axis.startReliefUsed && fabs(axis.output)>=bias &&
         (recentPosition(axis,2)-axis.travelStartPosition)*axis.travelDirection>=_settings.number(MP_TRAVEL_START_EARLY_PROGRESS_DEG))
      {
        axis.startReliefUsed=true; axis.startReliefStartedMs=nowMs;
        axis.output=axis.travelDirection*fmin(fabs(axis.output),running);
      }
      if(axis.startReliefUsed && uint32_t(nowMs-axis.startReliefStartedMs)<_settings.milliseconds(MP_TRAVEL_START_RELIEF_MS))
        return axis.travelDirection*running;
      return axis.travelDirection*(axis.travelDirection<0 ? _settings.number(MP_TRAVEL_DOWN_START_PWM_MAX) : _settings.number(MP_TRAVEL_PWM_MAX));
    }
  }
  // Keep the measured running bias for startup/slow feedback, but allow a
  // bounded reduction while moving. A hard bias floor prevented the speed
  // loop from reducing the real unloaded velocity to its requested value.
  double directedVelocity=fmax(velocity*axis.travelDirection,axis.velocity*axis.travelDirection);
  double bias=axis.travelDirection>0 ? axis.configuration.positiveRunningPwm : axis.configuration.negativeRunningPwm;
  double reduction=axis.travelDirection<0 ? _settings.number(MP_TRAVEL_DOWN_PWM_REDUCTION_MAX) : _settings.number(MP_TRAVEL_PWM_REDUCTION_MAX);
  double minimum=fmax(1.0,bias-reduction);
  if(axis.jog) minimum=_settings.number(MP_JOG_MIN_PWM);
  // Restore the running bias before speed collapses into static friction.
  // The short slope reacts before the long quiet window; alternating sample
  // noise cancels over its two intervals and cannot reset any watchdog.
  double recoverySpeed=fmax(_settings.number(MP_VELOCITY_EXIT_DPS),_settings.number(MP_TRAVEL_START_PRELOAD_SPEED_FRACTION)*axis.travelSpeed);
  if(axis.jog) recoverySpeed=fmax(_settings.number(MP_VELOCITY_EXIT_DPS),_settings.number(MP_TRAVEL_START_PRELOAD_SPEED_FRACTION)*axis.travelSpeed);
  double shortVelocity=windowVelocity(axis,nowMs,(axis.jog || axis.manualTarget || axis.positioningTarget) ? (uint8_t)_settings.values[MP_JOG_VELOCITY_SAMPLES] : 3)*axis.travelDirection;
  bool recovering=shortVelocity<=recoverySpeed;
  // A weak uphill short slope must not wait for an older fast window to
  // decay before restoring torque during the bounded first-degree handoff.
  // Later cruise preserves overspeed evidence so noise cannot add torque to
  // an already fast drive. Negative noise never exceeds a stopped correction.
  bool earlyUpwardHandoff=axis.travelDirection>0 &&
    (recentPosition(axis,3)-axis.travelStartPosition)<_settings.number(MP_TRAVEL_UP_HANDOFF_PROGRESS_DEG);
  if(axis.travelDirection>0 && recovering && (earlyUpwardHandoff || directedVelocity<=axis.travelSpeed))
    directedVelocity=fmax(0.0,shortVelocity);
  if(axis.jog || axis.manualTarget || axis.positioningTarget) directedVelocity=fmax(0.0,shortVelocity);
  double speedError=axis.travelSpeed-directedVelocity;
  if((axis.manualTarget || axis.positioningTarget) && axis.travelDirection<0) minimum=fmax(minimum,bias-_settings.number(MP_TRAVEL_PWM_REDUCTION_MAX));
  if(recovering) minimum=bias;
  // Downhill feedback needs one fresh window before fast positive recovery.
  // Unwind downhill overspeed promptly without reversing the motor.
  bool recoveryReady=axis.travelDirection>0 || uint32_t(nowMs-axis.driveStageStartedMs)>=MOTION_FEEDBACK_HISTORY_LENGTH*MOTION_CONTROL_PERIOD_MS;
  double integralGain=(recoveryReady && recovering && speedError>0) || speedError<0 ? _settings.number(MP_TRAVEL_RECOVERY_INTEGRAL_GAIN) : _settings.number(MP_TRAVEL_BASE_I_GAIN);
  if(axis.travelDirection>0 && speedError<0) integralGain=_settings.number(MP_TRAVEL_UP_UNWIND_INTEGRAL_GAIN);
  if(axis.jog) integralGain=recovering && speedError>0 ? _settings.number(MP_TRAVEL_RECOVERY_INTEGRAL_GAIN) :
    axis.jogVelocity && speedError<0 ? jogVelocityGain(axis,MP_JOG_I_GAIN,MP_JOG_VELOCITY_UNWIND) : _settings.number(MP_JOG_I_GAIN);
  // Loaded upward travel must be able to use the existing output cap.
  double outputMaximum=_settings.number(MP_TRAVEL_PWM_MAX);
  if(axis.travelDirection<0 &&
     (recentPosition(axis,3)-axis.travelStartPosition)*axis.travelDirection<_settings.number(MP_TRAVEL_DOWN_START_CONFIRM_PROGRESS_DEG))
    outputMaximum=_settings.number(MP_TRAVEL_DOWN_START_PWM_MAX);
  double integralMaximum=axis.travelDirection>0 ? outputMaximum-bias : fmin(_settings.number(MP_TRAVEL_DOWN_I_MAX),outputMaximum-bias);
  if(axis.jogVelocity) integralMaximum+=_settings.number(MP_JOG_VELOCITY_P_GAIN)*axis.jogMaximumSpeed;
  double integralMinimum=axis.travelDirection<0 ? -_settings.number(MP_TRAVEL_DOWN_PWM_REDUCTION_MAX) : -_settings.number(MP_TRAVEL_UP_I_REDUCTION_MAX);
  if(axis.jog) integralMinimum=_settings.number(MP_JOG_MIN_PWM)-bias;
  if(axis.jog && !axis.jogVelocity && speedError<-_settings.number(MP_VELOCITY_EXIT_DPS)) axis.integral=fmin(axis.integral,0.0);
  double candidate=bounded(axis.integral+integralGain*speedError*dt,integralMinimum,integralMaximum);
  // Legacy jog discards stored breakaway torque on overspeed. Absolute
  // velocity mode unwinds it gradually to preserve the continuous handoff.
  if(axis.jog && !axis.jogVelocity && speedError<-_settings.number(MP_VELOCITY_EXIT_DPS)) candidate=fmin(candidate,0.0);
  double proportionalGain=speedError<0 ? _settings.number(MP_TRAVEL_OVERSPEED_P_GAIN) :
    (axis.travelDirection>0 && recovering ? _settings.number(MP_TRAVEL_UP_RECOVERY_PROPORTIONAL_GAIN) : _settings.number(MP_TRAVEL_LOW_P_GAIN));
  if(axis.jog) proportionalGain=axis.jogVelocity ? jogVelocityGain(axis,MP_TRAVEL_LOW_P_GAIN,MP_JOG_VELOCITY_P_GAIN) : _settings.number(MP_TRAVEL_LOW_P_GAIN);
  double desired=bias+proportionalGain*speedError+candidate;
  // Do not wind up against either output limit; retain the ability to raise
  // running torque again before the independent stall watchdog expires.
  if((desired>=minimum || speedError>0) && (desired<=outputMaximum || speedError<0))
    axis.integral=candidate;
  desired=bias+proportionalGain*speedError+axis.integral;
  return axis.travelDirection*bounded(desired,minimum,outputMaximum);
}

double MotionController::controlJogPan(Axis &axis,uint32_t nowMs,double dt)
{
  bool finite=axis.manualTarget || axis.positioningTarget;
  double tolerance=_settings.number(axis.manualTarget ? MP_MANUAL_PAN_TOLERANCE : MP_POSITION_ENTER_DEG);
  if(finite && (axis.driveStage==DRIVE_START || axis.driveStage==DRIVE_CRUISE) && fabs(axis.target-meanPosition(axis))<=tolerance &&
     fabs(quietVelocity(axis,nowMs))<=_settings.number(MP_VELOCITY_ENTER_DPS) && span(axis)<=_settings.number(MP_QUIET_SPAN_DEG))
  {
    axis.driveStage=DRIVE_SETTLE; axis.profileComplete=true; axis.correcting=false;
    axis.reference=axis.target; axis.referenceVelocity=axis.referenceAcceleration=0; axis.output=axis.integral=0;
    return 0;
  }
  if(axis.driveStage==DRIVE_SETTLE) return 0;
  if(axis.driveStage==DRIVE_BRAKE) {
    if(finite) {
      if(uint32_t(nowMs-axis.driveStageStartedMs)>=_settings.milliseconds(MP_TRAVEL_SETTLE_TIMEOUT_MS))
      { fail("MOTION_SETTLE_TIMEOUT"); return 0; }
      bool resting=fabs(quietVelocity(axis,nowMs))<=_settings.number(MP_VELOCITY_ENTER_DPS) && span(axis)<=_settings.number(MP_QUIET_SPAN_DEG);
      if(!resting) axis.brakeQuiet=false;
      else {
        if(!axis.brakeQuiet) { axis.brakeQuiet=true; axis.brakeQuietStartedMs=nowMs; }
        if(uint32_t(nowMs-axis.brakeQuietStartedMs)>=_settings.milliseconds(MP_CONFIRM_MS)) {
          if(fabs(axis.target-meanPosition(axis))>tolerance+1e-9)
          { fail("MOTION_TARGET_MISSED"); return 0; }
          axis.driveStage=DRIVE_SETTLE;
        }
      }
    }
    return 0;
  }
  bool continuous=axis.jogVelocity || finite;
  double shortVelocity=windowVelocity(axis,nowMs,(uint8_t)_settings.values[MP_JOG_VELOCITY_SAMPLES])*axis.travelDirection;
  // Use the recent position for braking: the full rest window lags by
  // almost 0.1 s, which overshoots short loaded moves during breakaway.
  double remaining=(axis.target-recentPosition(axis,3))*axis.travelDirection;
  // Finite moves slow down before the endpoint; only the short passive coast
  // remains here. The longer Pan lookahead sets the approach speed below.
  double speed=fmax(0.0,fmax(shortVelocity,axis.velocity*axis.travelDirection));
  double coast=finite ? speed*speed/(2*_settings.number(MP_PAN_TARGET_COAST_DECELERATION)) :
    speed*_settings.number(continuous ? MP_PAN_JOG_BRAKE_LEAD : MP_TILT_BRAKE_LOOKAHEAD_SECONDS);
  if(finite) {
    unsigned int count=(unsigned int)_settings.values[MP_PAN_TARGET_PREDICTION_SAMPLES];
    if(count>axis.historyCount) count=axis.historyCount;
    double times[10],positions[10],positionNow,velocityNow;
    for(unsigned int i=0;i<count;++i) {
      unsigned int index=(axis.historyIndex+MOTION_HISTORY_LENGTH-1-i)%MOTION_HISTORY_LENGTH;
      times[i]=-(double)uint32_t(nowMs-axis.historyTimes[index])/1000.0;positions[i]=axis.history[index];
    }
    if(PanStopModel::estimate(times,positions,count,positionNow,velocityNow)) {
      double velocity=fmax(0.0,velocityNow*axis.travelDirection);
      coast=fmax(0.0,(positionNow-recentPosition(axis,3))*axis.travelDirection+
        velocity*velocity/(2*_settings.number(MP_PAN_TARGET_COAST_DECELERATION))+
        velocity*_settings.number(MP_PAN_TARGET_COAST_DELAY));
    }
  }
  if(remaining<=tolerance*0.5+coast)
  {
    axis.driveStage=DRIVE_BRAKE; axis.driveStageStartedMs=nowMs; axis.brakeQuiet=false;
    axis.output=axis.integral=0; axis.profileComplete=true;
    if(finite) { axis.reference=axis.target; axis.referenceVelocity=axis.referenceAcceleration=0; }
    return 0;
  }
  double bias=_settings.number(MP_PAN_JOG_RUNNING_PWM), maximum=_settings.number(MP_PAN_JOG_START_PWM);
  if(axis.driveStage==DRIVE_START)
  {
    if(uint32_t(nowMs-axis.driveStageStartedMs)>=_settings.milliseconds(MP_TRAVEL_START_TIMEOUT_MS))
    { fail("MOTION_STALLED"); return 0; }
    if((meanPosition(axis)-axis.travelStartPosition)*axis.travelDirection<-_settings.number(MP_POSITION_EXIT_DEG) &&
        quietVelocity(axis,nowMs)*axis.travelDirection<-_settings.number(MP_VELOCITY_EXIT_DPS)) return 0;
    double position=continuous ? recentPosition(axis,(uint8_t)_settings.values[MP_JOG_VELOCITY_SAMPLES]) : axis.position;
    double confirmation=_settings.number(continuous ? MP_JOG_VELOCITY_START_PROGRESS : MP_TRAVEL_START_PROGRESS_DEG);
    if(finite) confirmation=fmax(confirmation,2*_settings.number(MP_QUIET_SPAN_DEG));
    if((position-axis.travelStartPosition)*axis.travelDirection<confirmation || shortVelocity<=_settings.number(MP_VELOCITY_EXIT_DPS))
      return axis.travelDirection*maximum;
    axis.driveStage=DRIVE_CRUISE; axis.driveStageStartedMs=nowMs;
    if(continuous) {
      // Remove bounded excess breakaway torque after measured movement.
      // Keep the output itself continuous; the normal slew removes the
      // excess. Carrying the whole start peak into a short target overshoots.
      double handoff=fabs(axis.output);
      if(finite) handoff=fmax(bias,handoff-_settings.number(MP_PAN_TARGET_START_RELIEF_PWM));
      axis.integral=handoff-bias-
        (finite ? _settings.number(MP_PAN_JOG_P_GAIN) : jogVelocityGain(axis,MP_TRAVEL_LOW_P_GAIN,MP_JOG_VELOCITY_P_GAIN))*(axis.travelSpeed-shortVelocity);
    }
    else { axis.integral=0; axis.output=axis.travelDirection*fmin(fabs(axis.output),bias); }
  }
  double error=axis.travelSpeed-fmax(0.0,shortVelocity);
  bool recovering=shortVelocity<=fmax(_settings.number(MP_VELOCITY_EXIT_DPS),_settings.number(MP_TRAVEL_START_PRELOAD_SPEED_FRACTION)*axis.travelSpeed);
  // Encoder velocity noise alone must not switch finite targets to a
  // high-gain loop. Require sustained missing position progress, then blend
  // the integral gain without resetting torque or changing the PWM slew.
  double gain=finite ? _settings.number(MP_JOG_I_GAIN) :
    recovering && error>0 ? _settings.number(MP_TRAVEL_RECOVERY_INTEGRAL_GAIN) :
    continuous && error<0 ? jogVelocityGain(axis,MP_JOG_I_GAIN,MP_JOG_VELOCITY_UNWIND) : _settings.number(MP_JOG_I_GAIN);
  if(finite && error>0 && axis.progressWatch) {
    double delay=_settings.milliseconds(MP_PAN_TARGET_RECOVERY_DELAY_MS);
    double blend=bounded((double(uint32_t(nowMs-axis.progressStartedMs))-delay)/delay,0.0,1.0);
    gain+=blend*fmax(0.0,_settings.number(MP_PAN_TARGET_RECOVERY_I_GAIN)-gain);
  }
  double minimum=!finite && recovering ? bias : _settings.number(MP_JOG_MIN_PWM);
  if(!continuous && error<-_settings.number(MP_VELOCITY_EXIT_DPS)) axis.integral=fmin(axis.integral,0.0);
  double integralMaximum=maximum-bias+(continuous ? _settings.number(MP_JOG_VELOCITY_P_GAIN)*axis.jogMaximumSpeed : 0.0);
  double candidate=bounded(axis.integral+gain*error*dt,_settings.number(MP_JOG_MIN_PWM)-bias,integralMaximum);
  if(!continuous && error<-_settings.number(MP_VELOCITY_EXIT_DPS)) candidate=fmin(candidate,0.0);
  double proportional=finite ? _settings.number(MP_PAN_JOG_P_GAIN) : continuous ? jogVelocityGain(axis,MP_TRAVEL_LOW_P_GAIN,MP_JOG_VELOCITY_P_GAIN) : _settings.number(MP_PAN_JOG_P_GAIN);
  double desired=bias+proportional*error+candidate;
  if((desired>=minimum || error>0) && (desired<=maximum || error<0)) axis.integral=candidate;
  return axis.travelDirection*bounded(bias+proportional*error+axis.integral,minimum,maximum);
}

void MotionController::beginFine(Axis &axis,uint32_t nowMs,double velocity)
{
  axis.fineUsed=true; axis.fineStartedMs=nowMs; axis.fineStartPosition=meanPosition(axis);
  axis.fineVelocity=velocity; axis.fineAcceleration=0;
  axis.driveStage=DRIVE_FINE; axis.driveStageStartedMs=nowMs;
  axis.profileComplete=true; axis.correcting=true; axis.quiet=false;
  axis.progressWatch=axis.reversalWatch=axis.reversing=false;
  double bias=axis.travelDirection>0 ? axis.configuration.positiveRunningPwm : axis.configuration.negativeRunningPwm;
  axis.integral=bounded(fabs(axis.output)-bias,-bias+1,_settings.number(MP_FINE_MAX_PWM)-bias);
}

double MotionController::controlFine(Axis &axis,uint32_t nowMs,double dt)
{
  double remaining=(axis.target-recentPosition(axis,3))*axis.travelDirection;
  if(uint32_t(nowMs-axis.fineStartedMs)>_settings.milliseconds(MP_FINE_MAX_DURATION_MS) ||
     fabs(meanPosition(axis)-axis.fineStartPosition)>_settings.number(MP_FINE_MAX_DISTANCE))
  { fail("MOTION_FINE_TIMEOUT"); return 0; }
  double velocity=fmax(0.0,quietVelocity(axis,nowMs)*axis.travelDirection);
  double lead=velocity*_settings.number(MP_TILT_BRAKE_LOOKAHEAD_SECONDS);
  if(remaining<=_settings.number(MP_FINE_STOP_ERROR_DEG)+lead)
  {
    // End once by removing drive power; passive coast uses the same
    // committed brake as normal travel. Never ramp torque through stiction.
    axis.driveStage=DRIVE_BRAKE; axis.driveStageStartedMs=nowMs; axis.brakeQuiet=false;
    axis.integral=axis.output=0; axis.reference=axis.target; axis.referenceVelocity=axis.referenceAcceleration=0;
    axis.progressWatch=false; return 0;
  }
  double distance=fmax(0.0,remaining-_settings.number(MP_FINE_STOP_ERROR_DEG));
  double requested=fmin(_settings.number(MP_FINE_MAX_VELOCITY),sqrt(2*_settings.number(MP_FINE_MAX_ACCELERATION)*distance));
  // Critically damped velocity transition: continuous acceleration with a
  // bounded jerk. Do not reset acceleration when crossing a speed setpoint.
  double response=fmin(_settings.number(MP_FINE_MAX_JERK)/_settings.number(MP_FINE_MAX_ACCELERATION),1.0/dt);
  double jerk=response*response*(requested-axis.fineVelocity)-2*response*axis.fineAcceleration;
  axis.fineAcceleration=bounded(axis.fineAcceleration+bounded(jerk,
    -_settings.number(MP_FINE_MAX_JERK),_settings.number(MP_FINE_MAX_JERK))*dt,
    -_settings.number(MP_FINE_MAX_ACCELERATION),_settings.number(MP_FINE_MAX_ACCELERATION));
  axis.fineVelocity=fmax(0.0,axis.fineVelocity+axis.fineAcceleration*dt);
  axis.reference=axis.target; axis.referenceVelocity=axis.travelDirection*axis.fineVelocity;
  axis.referenceAcceleration=axis.travelDirection*axis.fineAcceleration;
  double bias=axis.travelDirection>0 ? axis.configuration.positiveRunningPwm : axis.configuration.negativeRunningPwm;
  double maximum=_settings.number(MP_FINE_MAX_PWM);
  double error=axis.fineVelocity-velocity;
  double candidate=bounded(axis.integral+axis.configuration.velocityIntegralGain*error*dt,-bias+1,maximum-bias);
  double desired=bias+axis.configuration.velocityGain*error+candidate;
  if((desired<=maximum || error<0) && (desired>=0 || error>0)) axis.integral=candidate;
  desired=bias+axis.configuration.velocityGain*error+axis.integral;
  return axis.travelDirection*bounded(desired,0,maximum);
}

void MotionController::control(Axis &axis, uint32_t nowMs, double dt)
{
  if (!axis.active) { axis.output = 0; return; }
  bool travel=stagedTravel(axis);
  bool manualPan=(axis.manualTarget || axis.positioningTarget) && &axis==&_axes[0] && !travel;
  double positionEnter=travel ? _settings.number(MP_TRAVEL_POSITION_ENTER_DEG) : _settings.number(MP_POSITION_ENTER_DEG);
  double positionExit=travel ? _settings.number(MP_TRAVEL_POSITION_EXIT_DEG) : _settings.number(MP_POSITION_EXIT_DEG);
  if(axis.manualTarget && &axis==&_axes[0]) { positionEnter=_settings.number(MP_MANUAL_PAN_TOLERANCE); positionExit=fmax(positionExit,positionEnter); }
  if((axis.jog || manualPan || axis.positioningTarget) && !axis.profileComplete)
  {
    double acceleration=_settings.number(manualPan || (axis.jogVelocity && &axis==&_axes[0]) ? MP_PAN_JOG_ACCELERATION : MP_JOG_ACCELERATION);
    if(axis.positioningAcceleration>0) acceleration=axis.positioningAcceleration;
    double remaining=fmax(0.0,(axis.target-axis.reference)*axis.travelDirection);
    double cap=fmin(axis.jogMaximumSpeed,sqrt(2.0*acceleration*remaining));
    if(manualPan) {
      double distance=fmax(0.0,(axis.target-meanPosition(axis))*axis.travelDirection-positionEnter*0.5);
      // A loaded axis can lag behind its virtual reference. Reaching that
      // reference endpoint must not request zero speed before actual arrival.
      cap=fmin(axis.jogMaximumSpeed,sqrt(2.0*acceleration*distance));
      cap=fmin(cap,distance/_settings.number(MP_PAN_JOG_BRAKE_LEAD));
      if(fabs(axis.target-meanPosition(axis))>positionEnter)
        cap=fmin(axis.jogMaximumSpeed,fmax(cap,_settings.number(MP_PAN_TARGET_MIN_APPROACH_VELOCITY)));
    }
    double before=axis.travelSpeed;
    axis.travelSpeed=bounded(cap,before-acceleration*dt,before+acceleration*dt);
    axis.referenceVelocity=axis.travelDirection*axis.travelSpeed;
    axis.referenceAcceleration=axis.travelDirection*(axis.travelSpeed-before)/dt;
    double lead=_settings.number(MP_TRACKING_MAX_DEG);
    double next=axis.reference+axis.referenceVelocity*dt;
    axis.reference=axis.travelDirection>0 ? fmin(axis.target,fmin(next,axis.position+lead)) : fmax(axis.target,fmax(next,axis.position-lead));
    if(!manualPan && remaining<0.001 && axis.travelSpeed<0.02) { axis.reference=axis.target; axis.referenceVelocity=0; axis.profileComplete=true; }
  }
  else if (!axis.profileComplete)
  {
    int8_t travelDirection = polarity(axis.referenceVelocity);
    if (travelDirection == 0) travelDirection = polarity(axis.target - axis.reference);
    // Bound reference lead only when the output is behind in travel direction.
    // An output ahead of the profile must let it catch up; otherwise a frozen
    // position with nonzero reference velocity can balance permanently.
    bool lagging = (axis.reference - axis.position) * travelDirection > _settings.number(MP_TRACKING_MAX_DEG);
    if (!lagging) axis.profileTime += dt; // Allow a bounded lead to engage a previously unloaded gear.
    axis.profile.evaluate(axis.profileTime, axis.reference, axis.referenceVelocity, axis.referenceAcceleration);
    axis.profileComplete = axis.profileTime >= axis.profile.duration();
  }
  // An already stationary output inside target tolerance has completed the
  // physical move. A slow nominal curve must not keep applying torque or
  // delay endpoint capture; retain the normal independent quiet dwell.
  if(!travel && !axis.jog && !axis.profileComplete &&
     fabs(axis.target-meanPosition(axis))<=positionEnter+1e-9 &&
     fabs(quietVelocity(axis,nowMs))<=_settings.number(MP_VELOCITY_ENTER_DPS) &&
     span(axis)<=_settings.number(MP_QUIET_SPAN_DEG))
  {
    axis.reference=axis.target; axis.referenceVelocity=axis.referenceAcceleration=0;
    axis.profileTime=axis.profile.duration(); axis.profileComplete=true;
    axis.integral=0; axis.correcting=false;
  }
  // Near the target use the same history that establishes rest. A single
  // encoder outlier must not re-engage stiction or demand a new reversal.
  double error = axis.target - ((travel || axis.profileComplete) ?
    meanPosition(axis) : axis.position);
  double holdVelocity = quietVelocity(axis, nowMs);
  if (axis.profileComplete && !travel)
  {
    if (!axis.correcting && (fabs(error) > positionExit || fabs(holdVelocity) > _settings.number(MP_VELOCITY_EXIT_DPS)))
      axis.correcting = true;
    if (axis.correcting && fabs(error) <= positionEnter && fabs(holdVelocity) <= _settings.number(MP_VELOCITY_ENTER_DPS))
      axis.correcting = false;
  }
  double requestedVelocity=axis.referenceVelocity;
  bool velocityPi=axis.configuration.velocityIntegralGain>0;
  if(velocityPi && axis.correcting && !travel)
  {
    double predictedPosition=axis.position+axis.velocity*_settings.number(MP_TILT_BRAKE_LOOKAHEAD_SECONDS);
    if(axis.profileComplete)
      requestedVelocity=fabs(error)<=_settings.number(MP_POSITION_ENTER_DEG) ? 0 :
        bounded(axis.configuration.positionVelocityGain*(error-axis.velocity*_settings.number(MP_TILT_BRAKE_LOOKAHEAD_SECONDS)),-_settings.number(MP_FINAL_MAX_DPS),_settings.number(MP_FINAL_MAX_DPS));
    else requestedVelocity+=bounded(axis.configuration.positionVelocityGain*(axis.reference-predictedPosition),-_settings.number(MP_TRACKING_MAX_DPS),_settings.number(MP_TRACKING_MAX_DPS));
    double actualError=axis.target-axis.position;
    bool brakingToTarget=fabs(axis.velocity)>_settings.number(MP_VELOCITY_EXIT_DPS) && actualError*axis.velocity>0 &&
      fabs(actualError)<=_settings.number(MP_POSITION_ENTER_DEG)+fabs(axis.velocity)*_settings.number(MP_TILT_BRAKE_LOOKAHEAD_SECONDS);
    if(brakingToTarget) { requestedVelocity=0; axis.integral=0; }
    int8_t intendedDirection=polarity(axis.target-meanPosition(axis));
    if(fabs(axis.target-meanPosition(axis))>_settings.number(MP_POSITION_EXIT_DEG))
    {
      if(axis.integralDirection && intendedDirection && axis.integralDirection!=intendedDirection) axis.integral=0;
      if(intendedDirection) axis.integralDirection=intendedDirection;
    }
    double velocityError=requestedVelocity-axis.velocity;
    // Integrate only while torque has reserve or the error unwinds it. The
    // independent +/-24 bound also covers the reversal torque limitation.
    if(fabs(axis.output)<axis.configuration.maximumPwm || velocityError*axis.output<0)
      axis.integral=bounded(axis.integral+axis.configuration.velocityIntegralGain*velocityError*dt,-_settings.number(MP_VELOCITY_INTEGRAL_MAX),_settings.number(MP_VELOCITY_INTEGRAL_MAX));
  }
  double frictionPwm=requestedVelocity>=0 ? axis.configuration.frictionFeedforwardPwm :
    axis.configuration.negativeFrictionFeedforwardPwm;
  double desired = axis.correcting ? (velocityPi ? (requestedVelocity==0 ? 0 : axis.integral) :
      axis.configuration.proportionalGain*(axis.reference-(axis.profileComplete ? meanPosition(axis) : axis.position))) +
    axis.configuration.velocityGain*(requestedVelocity-axis.velocity) +
    frictionPwm*bounded(requestedVelocity/_settings.number(MP_FRICTION_FULL_DPS),-1,1) +
    axis.configuration.velocityFeedforwardGain*requestedVelocity : 0;
  if(!travel && !axis.jog && !manualPan && !velocityPi && axis.configuration.trackingIntegralGain>0)
  {
    // A frozen profile bounds tracking error but cannot overcome a sustained
    // load with PD alone. Accumulate bounded torque from filtered error;
    // never wind up against unconfirmed backlash or either output cap.
    double targetError=axis.target-meanPosition(axis);
    bool finalBraking=axis.profileComplete && targetError*axis.velocity>0 &&
      fabs(axis.velocity)>_settings.number(MP_VELOCITY_ENTER_DPS) &&
      fabs(targetError)<=_settings.number(MP_POSITION_ENTER_DEG)+fabs(axis.velocity)*_settings.number(MP_TILT_BRAKE_LOOKAHEAD_SECONDS);
    // Release breakaway torque as actual motion enters the stopping window.
    // Keeping it until confirmed rest drives a low-friction load past target.
    if(!axis.correcting || (axis.profileComplete && targetError*axis.integral<0) ||
       (fabs(targetError)>_settings.number(MP_POSITION_EXIT_DEG) && targetError*axis.integral<0) || finalBraking) axis.integral=0;
    else if(!axis.reversing)
    {
      double trackingError=axis.reference-meanPosition(axis);
      double candidate=bounded(axis.integral+axis.configuration.trackingIntegralGain*trackingError*dt,
        -_settings.number(MP_TRACKING_INTEGRAL_PWM_MAX),_settings.number(MP_TRACKING_INTEGRAL_PWM_MAX));
      double proposed=desired+candidate;
      if((proposed<=axis.configuration.maximumPwm || trackingError<0) &&
         (proposed>=-axis.configuration.maximumPwm || trackingError>0)) axis.integral=candidate;
    }
    desired+=axis.integral;
  }
  if(travel) { desired=controlTravel(axis,nowMs,dt); if(_fault) return; }
  else if(axis.jog || manualPan) { desired=controlJogPan(axis,nowMs,dt); if(_fault) return; }
  desired = bounded(desired, -axis.configuration.maximumPwm, axis.configuration.maximumPwm);
  int8_t direction = polarity(desired);
  double filteredTargetError=axis.target-meanPosition(axis);
  bool requestedTravelDirection=fabs(filteredTargetError)>_settings.number(MP_POSITION_EXIT_DEG) &&
    polarity(filteredTargetError)==direction;
  // A transient braking torque is not an enduring request to reverse travel.
  // Returning to the confirmed drive direction cancels that pending reversal.
  if (axis.reversing && direction != 0 && direction == axis.driveDirection && direction != axis.reversalDirection)
  {
    axis.reversing = false; axis.reversalWatch = false; axis.progressWatch = false;
  }
  if (!axis.reversing && direction != 0 && direction != axis.driveDirection &&
      (!travel || axis.driveStage!=DRIVE_BRAKE) && requestedTravelDirection)
  {
    axis.reversing = true; axis.reversalDirection = direction;
    axis.reversalPosition = meanPosition(axis); axis.reversalStartedMs = nowMs; axis.reversalWatch = false;
  }
  // Braking torque near an approached target does not request reverse travel.
  // Only an enduring target beyond the deadband arms the backlash timeout.
  bool intendsReversal=fabs(filteredTargetError)>_settings.number(MP_POSITION_EXIT_DEG) &&
    polarity(filteredTargetError)==axis.reversalDirection;
  if(axis.reversing && !intendsReversal) axis.reversalWatch=false;
  if (axis.reversing)
  {
    if ((meanPosition(axis) - axis.reversalPosition) * axis.reversalDirection >= _settings.number(MP_PROGRESS_DEG))
    {
      axis.driveDirection = axis.reversalDirection;
      axis.reversing = false; axis.progressWatch = false;
    }
    else if (desired == 0 && fabs(error) <= _settings.number(MP_POSITION_EXIT_DEG) && axis.profileComplete)
      axis.reversing = false;
    else if (axis.reversalWatch && (uint32_t)(nowMs - axis.reversalStartedMs) >= _settings.milliseconds(MP_REVERSAL_TIMEOUT_MS))
    { fail("REVERSAL_TIMEOUT"); return; }
    else desired = bounded(desired, -_settings.number(MP_REVERSAL_PWM_MAX), _settings.number(MP_REVERSAL_PWM_MAX));
  }
  else if(direction!=0 && direction!=axis.driveDirection && !requestedTravelDirection)
    desired=bounded(desired,-_settings.number(MP_REVERSAL_PWM_MAX),_settings.number(MP_REVERSAL_PWM_MAX));
  // Never jump directly from one polarity to the other, even for a braking correction.
  double slew=travel && axis.fineUsed && (axis.driveStage==DRIVE_FINE || axis.driveStage==DRIVE_BRAKE) ?
    _settings.number(MP_FINE_PWM_SLEW) : _settings.number(MP_PWM_SLEW_PER_SECOND);
  // Jog raises torque gently; after breakaway it may remove excess torque
  // at the normal configured rate instead of continuing to accelerate.
  if((axis.jog || manualPan || ((axis.manualTarget || axis.positioningTarget) && travel)) && fabs(desired)>fabs(axis.output)) slew=fmin(slew,_settings.number(MP_JOG_PWM_SLEW));
  double step = slew * dt;
  if (axis.output * desired < 0) desired = 0;
  axis.output += bounded(desired - axis.output, -step, step);
  if ((axis.position <= axis.minimum && axis.output < 0) || (axis.position >= axis.maximum && axis.output > 0))
    axis.output = 0;
  if (axis.reversing && intendsReversal && !axis.reversalWatch && (axis.profileComplete ||
      (polarity(axis.output) == axis.reversalDirection && fabs(axis.output) >= _settings.number(MP_STALL_PWM_MIN))))
  { axis.reversalWatch = true; axis.reversalStartedMs = nowMs; }

  double progressPosition=meanPosition(axis);
  // The velocity PI first builds bounded breakaway torque. Do not time out
  // that legitimate buildup while still below the measured start region.
  // Once there, filtered progress is required even for a final correction.
  double progressPwm=travel ? (axis.driveStage==DRIVE_FINE ? _settings.number(MP_SLOW_PROGRESS_PWM_MIN) : 0.5) : velocityPi ? _settings.number(MP_SLOW_PROGRESS_PWM_MIN) : _settings.number(MP_STALL_PWM_MIN);
  double progressError=travel || manualPan ? positionEnter : velocityPi ? _settings.number(MP_POSITION_ENTER_DEG) : _settings.number(MP_POSITION_EXIT_DEG);
  bool needsProgress = (!travel || axis.driveStage==DRIVE_CRUISE || axis.driveStage==DRIVE_FINE) && fabs(axis.output) >= progressPwm &&
    fabs(axis.target-progressPosition) > progressError;
  if (!needsProgress) axis.progressWatch = false;
  else if (!axis.progressWatch || (progressPosition - axis.progressPosition) * polarity(axis.output) >= _settings.number(MP_PROGRESS_DEG))
  {
    axis.progressWatch = true; axis.progressPosition = progressPosition; axis.progressStartedMs = nowMs;
  }
  else if ((uint32_t)(nowMs - axis.progressStartedMs) >= _settings.milliseconds(MP_NO_PROGRESS_MS))
  { fail("MOTION_STALLED"); return; }

  double positionWindow = !axis.correcting ? positionExit : positionEnter;
  double velocityWindow = !axis.correcting ? _settings.number(MP_VELOCITY_EXIT_DPS) : _settings.number(MP_VELOCITY_ENTER_DPS);
  bool quiet = axis.profileComplete && (!travel || axis.driveStage==DRIVE_SETTLE) &&
    !axis.reversing && fabs(error) <= positionWindow+1e-9 &&
    fabs(holdVelocity) <= velocityWindow && span(axis) <= _settings.number(MP_QUIET_SPAN_DEG);
  if (quiet && !axis.quiet) axis.quietStartedMs = nowMs;
  axis.quiet = quiet;
}

void MotionController::tick(uint32_t nowMs, double panPosition, double tiltPosition, bool sensorsValid, bool enabled)
{
  if (!enabled) disable(); // Safety permission applies even to a duplicate timestamp.
  if (!sensorsValid || !isfinite(panPosition) || !isfinite(tiltPosition))
  { _hasSample = false; fail("MOTION_SENSOR"); return; }
  uint32_t elapsed = (uint32_t)(nowMs - _lastSampleMs);
  if (_hasSample && elapsed == 0) return; // Duplicate sample does not advance anything.
  bool late = _hasSample && elapsed > MOTION_MAX_SAMPLE_GAP_MS;
  if (late && enabled && (_axes[0].active || _axes[1].active)) { fail("CONTROL_TIMEOUT"); return; }
  double dt = !_hasSample || late ? MOTION_CONTROL_PERIOD_MS / 1000.0 : elapsed / 1000.0;
  observe(_axes[0], panPosition, dt, !_hasSample || late, nowMs);
  observe(_axes[1], tiltPosition, dt, !_hasSample || late, nowMs);
  _lastSampleMs = nowMs; _hasSample = true; ++_sampleSequence;
  if (!enabled) return;
  if (_fault) return;
  control(_axes[0], nowMs, dt);
  if (!_fault) control(_axes[1], nowMs, dt);
}

#if defined(PANODUINO_MOTION_TRACE) || defined(PANODUINO_HOST_TEST)
MotionTraceSnapshot MotionController::traceSnapshot(bool tiltAxis) const
{
  const Axis &axis=_axes[tiltAxis ? 1 : 0];
  MotionTraceSnapshot result={_sampleSequence,_lastSampleMs,status(tiltAxis),
    windowVelocity(axis,_lastSampleMs,3),quietVelocity(axis,_lastSampleMs),axis.integral,
    meanPosition(axis),axis.profileTime,axis.progressWatch ? uint32_t(_lastSampleMs-axis.progressStartedMs) : 0,
    axis.reversing};
  return result;
}
#endif

void MotionController::disable()
{
  for (unsigned int i = 0; i < 2; ++i)
  {
    Axis &axis = _axes[i];
    axis.output = axis.integral = 0; axis.active = axis.quiet = axis.reversing = axis.progressWatch = axis.reversalWatch = false;
    axis.driveDirection = axis.integralDirection = 0;
    axis.driveStage=DRIVE_IDLE; axis.travelDirection=0; axis.startReliefUsed=false; axis.startReliefStartedMs=0; axis.jog=axis.jogVelocity=axis.manualTarget=axis.positioningTarget=false;
    axis.positioningAcceleration=0;
    axis.fineUsed=false; axis.fineStartedMs=0; axis.brakeQuiet=false; axis.brakeQuietStartedMs=0;
  }
  _fault = 0;
}

void MotionController::fail(const char *reason)
{ disable(); _fault = reason; }

void MotionController::resetFeedback()
{
  disable(); _hasSample = false;
  for (unsigned int i = 0; i < 2; ++i)
  {
    _axes[i].historyCount = _axes[i].historyIndex = 0;
    _axes[i].velocity = 0;
  }
}

bool MotionController::quietSince(uint32_t &sinceMs) const
{
  if (_fault || !_axes[0].active || !_axes[1].active || !_axes[0].quiet || !_axes[1].quiet) return false;
  uint32_t age = (uint32_t)(_lastSampleMs - _axes[0].quietStartedMs);
  uint32_t tiltAge = (uint32_t)(_lastSampleMs - _axes[1].quietStartedMs);
  sinceMs = _lastSampleMs - (age < tiltAge ? age : tiltAge);
  return true;
}

bool MotionController::passiveCaptureStable(uint32_t now,double pan,double tilt) const
{
  // Once drive power is removed, judge bounded position variation, not a
  // differentiated noise spike. Acquisition still uses strict quiet dwell.
  return feedbackFresh(now) && !_axes[0].active && !_axes[1].active &&
    span(_axes[0])<=_settings.number(MP_CAPTURE_SPAN_DEG) &&
    span(_axes[1])<=_settings.number(MP_CAPTURE_SPAN_DEG) &&
    fabs(meanPosition(_axes[0])-pan)<=_settings.number(MP_CAPTURE_PAN_DRIFT_DEG) &&
    fabs(meanPosition(_axes[1])-tilt)<=_settings.number(MP_CAPTURE_TILT_DRIFT_DEG);
}

bool MotionController::reached() const
{ uint32_t since; return quietSince(since) && (uint32_t)(_lastSampleMs - since) >= _settings.milliseconds(MP_CONFIRM_MS); }

uint8_t MotionController::confirmedSamples() const
{
  uint32_t since;
  if (!quietSince(since)) return 0;
  const uint32_t required = (_settings.milliseconds(MP_CONFIRM_MS) + MOTION_CONTROL_PERIOD_MS - 1) / MOTION_CONTROL_PERIOD_MS;
  uint32_t age = _lastSampleMs - since;
  uint32_t samples = age >= _settings.milliseconds(MP_CONFIRM_MS) ? required : age * required / _settings.milliseconds(MP_CONFIRM_MS);
  return (uint8_t)samples; // Equivalent confirmation progress, capped at the required value.
}

AxisMotionStatus MotionController::status(bool tiltAxis) const
{
  const Axis &axis = _axes[tiltAxis ? 1 : 0];
  AxisMotionStatus value;
  value.position = axis.position; value.velocity = axis.velocity; value.target = axis.target;
  value.reference = axis.reference; value.referenceVelocity = axis.referenceVelocity;
  value.referenceAcceleration = axis.referenceAcceleration;
  value.signedPwm = (int)lround(axis.output);
  value.phase = !axis.active ? MOTION_OFF : axis.reversing ? MOTION_REVERSING :
    axis.driveStage==DRIVE_FINE ? MOTION_FOLLOWING : axis.profileComplete ? MOTION_HOLDING : MOTION_FOLLOWING;
  value.withinTolerance = fabs(axis.target - (stagedTravel(axis) ? meanPosition(axis) : axis.position)) <=
    (stagedTravel(axis) ? (axis.quiet ? _settings.number(MP_TRAVEL_POSITION_EXIT_DEG) : _settings.number(MP_TRAVEL_POSITION_ENTER_DEG)) :
    (axis.quiet ? _settings.number(MP_POSITION_EXIT_DEG) : _settings.number(MP_POSITION_ENTER_DEG)));
  value.quiet = axis.quiet;
  value.driveStage=axis.driveStage;
  return value;
}















