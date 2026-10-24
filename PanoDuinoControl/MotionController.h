#ifndef PANODUINO_MOTION_CONTROLLER_H
#define PANODUINO_MOTION_CONTROLLER_H

#include <stdint.h>
#include "MotionConfiguration.h"
#include "MotionSettings.h"

// Quintic Bezier profile: continuous position, velocity and acceleration.
// Endpoint v/a are zero; derivative control polygons bound v/a/jerk.
class MotionProfile
{
public:
  MotionProfile();
  bool plan(double position, double velocity, double acceleration, double target,
            double minimum, double maximum, uint16_t speedPermille,
            const AxisMotionConfiguration &configuration);
  void evaluate(double seconds, double &position, double &velocity, double &acceleration) const;
  double duration() const { return _duration; }
private:
  double _points[6];
  double _duration;
};

enum MotionAxisPhase { MOTION_OFF, MOTION_FOLLOWING, MOTION_REVERSING, MOTION_HOLDING };
enum MotionDriveStage { DRIVE_IDLE, DRIVE_START, DRIVE_CRUISE, DRIVE_BRAKE, DRIVE_SETTLE, DRIVE_FINE };

struct AxisMotionStatus
{
  double position, velocity, target, reference, referenceVelocity, referenceAcceleration;
  int signedPwm;
  MotionAxisPhase phase;
  bool withinTolerance, quiet;
  MotionDriveStage driveStage;
};

#if defined(PANODUINO_MOTION_TRACE) || defined(PANODUINO_HOST_TEST)
struct MotionTraceSnapshot
{
  uint32_t sequence, sampleMs;
  AxisMotionStatus axis;
  double shortVelocity, quietVelocity, integral;
  double meanPosition, profileTime;
  uint32_t progressAgeMs;
  bool reversing;
};
#endif

class MotionController
{
public:
  MotionController();
  MotionController(const AxisMotionConfiguration &panConfiguration, const AxisMotionConfiguration &tiltConfiguration);
  void configureLimits(double panMinimum, double panMaximum, double tiltMinimum, double tiltMaximum);
  bool configure(const MotionSettings &settings);
  bool configurationAllowed() const
  { return !_axes[0].active && !_axes[1].active && (!_hasSample || feedbackResting(_lastSampleMs)); }
  const MotionSettings &settings() const { return _settings; }
  bool command(double panTarget, double tiltTarget, uint16_t panSpeed, uint16_t tiltSpeed);
  bool commandManual(double pan,double tilt,uint16_t panSpeed,uint16_t tiltSpeed);
  bool commandPositioning(double pan,double tilt,uint16_t panSpeed,uint16_t tiltSpeed);
  double targetVelocity(bool tilt,uint16_t tempo) const;
  bool commandColumnReturn(double pan,double tilt,uint16_t panSpeed,uint16_t tiltSpeed);
  bool commandJog(bool tiltAxis, int direction, uint16_t tempo);
  bool commandJogVelocity(bool tiltAxis, int direction, double velocityDps);
  bool jogLimitReached() const { return (_axes[0].jog && (_axes[0].quiet || _axes[0].driveStage==DRIVE_BRAKE)) || (_axes[1].jog && (_axes[1].quiet || _axes[1].driveStage==DRIVE_BRAKE)); }
  void tick(uint32_t nowMs, double panPosition, double tiltPosition, bool sensorsValid, bool enabled);
  void disable();
  void resetFeedback();
  const char *fault() const { return _fault; }
  bool reached() const;
  bool quietSince(uint32_t &sinceMs) const;
  uint32_t sampleSequence() const { return _sampleSequence; }
  bool feedbackFresh(uint32_t now) const { return _hasSample && uint32_t(now-_lastSampleMs)<=MOTION_MAX_SAMPLE_GAP_MS; }
  bool feedbackResting(uint32_t now) const
  { return feedbackFresh(now) && span(_axes[0])<=_settings.number(MP_POSITION_EXIT_DEG) && span(_axes[1])<=_settings.number(MP_POSITION_EXIT_DEG) &&
      quietVelocity(_axes[0],now)>=-_settings.number(MP_VELOCITY_EXIT_DPS) && quietVelocity(_axes[0],now)<=_settings.number(MP_VELOCITY_EXIT_DPS) &&
      quietVelocity(_axes[1],now)>=-_settings.number(MP_VELOCITY_EXIT_DPS) && quietVelocity(_axes[1],now)<=_settings.number(MP_VELOCITY_EXIT_DPS); }
  uint8_t confirmedSamples() const;
  bool passiveCaptureStable(uint32_t now,double pan,double tilt) const;
  AxisMotionStatus status(bool tiltAxis) const;
  uint32_t plannedMoveMaximumDurationMs() const;
#if defined(PANODUINO_MOTION_TRACE) || defined(PANODUINO_HOST_TEST)
  MotionTraceSnapshot traceSnapshot(bool tiltAxis) const;
#endif
private:
  struct Axis
  {
    MotionProfile profile;
    AxisMotionConfiguration configuration;
    double minimum, maximum, position, velocity, target;
    double reference, referenceVelocity, referenceAcceleration, profileTime;
    double output, integral, history[MOTION_HISTORY_LENGTH], reversalPosition, progressPosition;
    double travelStartPosition, travelSpeed, jogMaximumSpeed, positioningAcceleration;
    double fineVelocity, fineAcceleration, fineStartPosition;
    uint32_t historyTimes[MOTION_HISTORY_LENGTH];
    uint32_t quietStartedMs, reversalStartedMs, progressStartedMs;
    uint32_t driveStageStartedMs, startReliefStartedMs;
    uint32_t fineStartedMs, brakeQuietStartedMs;
    uint8_t historyCount, historyIndex;
    int8_t driveDirection, reversalDirection, integralDirection;
    int8_t travelDirection;
    MotionDriveStage driveStage;
    bool active, profileComplete, correcting, reversing, quiet, progressWatch, reversalWatch, startReliefUsed;
    bool fineUsed, brakeQuiet, jog, jogVelocity, manualTarget, positioningTarget;
  };
  void initializeAxes(const AxisMotionConfiguration &panConfiguration,const AxisMotionConfiguration &tiltConfiguration);
  void observe(Axis &axis, double position, double dt, bool resetVelocity, uint32_t nowMs);
  void control(Axis &axis, uint32_t nowMs, double dt);
  double controlTravel(Axis &axis, uint32_t nowMs, double dt);
  double controlJogPan(Axis &axis, uint32_t nowMs, double dt);
  double jogVelocityGain(const Axis &axis, MotionParameterId low, MotionParameterId high) const;
  void beginFine(Axis &axis,uint32_t nowMs,double velocity);
  double controlFine(Axis &axis,uint32_t nowMs,double dt);
  static bool stagedTravel(const Axis &axis);
  void fail(const char *reason);
  void targetMissed(Axis &axis);
  static double span(const Axis &axis);
  static double meanPosition(const Axis &axis);
  static double recentPosition(const Axis &axis, uint8_t maximumSamples);
  static double quietVelocity(const Axis &axis, uint32_t nowMs);
  static double windowVelocity(const Axis &axis, uint32_t nowMs, uint8_t maximumSamples);
  Axis _axes[2];
  MotionSettings _settings;
  uint32_t _lastSampleMs, _sampleSequence;
  bool _hasSample;
  const char *_fault;
  uint8_t _targetMissedAxes;
};

#endif
