#ifndef PANODUINO_BENCH_DRIVE_H
#define PANODUINO_BENCH_DRIVE_H

#include <stdint.h>
#include "MotionConfiguration.h"

struct BenchDriveSample
{
  uint32_t elapsedMs;
  int32_t panMdeg, tiltMdeg;
  int16_t signedPwm;
  uint8_t stage; // 0 constant, 1 starting, 2 cruising, 3 unpowered tail.
};

struct BenchDriveStatus
{
  uint16_t count;
  bool active, recording;
  const char *reason;
};

struct BenchOutputStatus
{
  int panAppliedPwm, tiltAppliedPwm, tiltPositivePin, tiltNegativePin;
  uint32_t tiltPeriod, tiltDuty, tiltCounter;
  bool tiltPeripheralMux;
};

// Diagnostic actuator pulses, exposed by the USB adapter only. No position
// feedback torque: this identifies drive response separately from regulation.
// Trace is buffered at control rate, avoiding live serial load during a pulse.
class BenchDrive
{
public:
  BenchDrive() : _panMin(-180000), _panMax(180000), _tiltMin(-45000), _tiltMax(30000),
    _count(0), _active(false), _recording(false), _tilt(false), _requested(0),
    _output(0), _start(0), _last(0), _end(0), _duration(0), _travel(0),
    _panStart(0), _tiltStart(0), _reason("NONE"), _boosted(false), _cruising(false), _cruisePwm(0) {}

  void configureLimits(int32_t panMin, int32_t panMax, int32_t tiltMin, int32_t tiltMax)
  { _panMin=panMin; _panMax=panMax; _tiltMin=tiltMin; _tiltMax=tiltMax; }

  bool start(bool tilt, int pwm, uint32_t duration, int32_t travel, uint32_t now,
             int32_t pan, int32_t tiltPosition, bool fresh)
  {
    if (_active || !fresh || pwm==0 || pwm < -40 || pwm > 40 || duration<100 || duration>3000 ||
        ((pwm < -30 || pwm > 30) && duration>500) ||
        travel<500 || travel>5000 || !inside(pan,tiltPosition)) return false;
    int64_t endpoint=(tilt ? tiltPosition : pan)+(int64_t)(pwm>0 ? travel : -travel);
    if (endpoint<(tilt ? _tiltMin : _panMin)+5000 || endpoint>(tilt ? _tiltMax : _panMax)-5000) return false;
    _tilt=tilt; _requested=pwm; _output=0; _start=_last=now;
    _boosted=_cruising=false; _cruisePwm=pwm;
    _duration=duration; _travel=travel; _panStart=pan; _tiltStart=tiltPosition;
    _count=0; _active=_recording=true; _reason="RUNNING";
    append(now,pan,tiltPosition);
    return true;
  }

  bool startCruise(bool tilt, int pwm, uint32_t duration, int32_t travel, uint32_t now,
                   int32_t pan, int32_t tiltPosition, bool fresh)
  {
    if(pwm==0 || pwm < -30 || pwm > 30 || duration<600 || travel<1000) return false;
    if(!start(tilt,pwm,duration,travel,now,pan,tiltPosition,fresh)) return false;
    _boosted=true; _requested=pwm>0 ? 40 : -40;
    _samples[0].stage=1;
    return true;
  }

  void tick(uint32_t now, int32_t pan, int32_t tilt, bool valid, bool enabled)
  {
    if (_active)
    {
      uint32_t dt=now-_last;
      if (!enabled) stop(now,"DISABLED");
      else if (!valid) stop(now,"SENSOR");
      else if (dt>MOTION_MAX_SAMPLE_GAP_MS) stop(now,"CONTROL_TIMEOUT");
      else if (!inside(pan,tilt)) stop(now,"LIMIT");
      else if (distance(pan,_panStart)>=_travel || distance(tilt,_tiltStart)>=_travel)
        stop(now,"TRAVEL_BOUND");
      else if ((_requested>0 && (int64_t)(_tilt ? tilt : pan)>=(int64_t)(_tilt ? _tiltMax : _panMax)-5000) ||
               (_requested<0 && (int64_t)(_tilt ? tilt : pan)<=(int64_t)(_tilt ? _tiltMin : _panMin)+5000))
        stop(now,"LIMIT_RESERVE");
      else if ((uint32_t)(now-_start)>=_duration) stop(now,"TIME_BOUND");
      else
      {
        if(_boosted && !_cruising)
        {
          int64_t progress=((int64_t)(_tilt ? tilt : pan)-(_tilt ? _tiltStart : _panStart))*(_requested>0 ? 1 : -1);
          if(progress<=-500) stop(now,"WRONG_DIRECTION");
          else if(progress>=500) { _cruising=true; _requested=_cruisePwm; }
          else if((uint32_t)(now-_start)>=500) stop(now,"START_TIMEOUT");
        }
        if(_active)
        {
          double step=MOTION_PWM_SLEW_PER_SECOND*dt/1000.0;
          double change=_requested-_output;
          _output+=change>step ? step : change < -step ? -step : change;
          // Independent high-power budget also covers a late start and the
          // down-ramp: output above 30 cannot persist beyond 500 ms.
          if(_boosted && (uint32_t)(now-_start)>=500)
            _output=_output>30 ? 30 : _output < -30 ? -30 : _output;
        }
      }
    }
    _last=now;
    if (_recording && valid)
    {
      if (_count && _samples[_count-1].elapsedMs==uint32_t(now-_start)) return;
      append(now,pan,tilt);
    }
    if (!_active && _recording && (uint32_t)(now-_end)>=500) _recording=false;
  }

  void stop(uint32_t now, const char *reason)
  { if (_active) { _active=false; _end=now; _reason=reason; } _output=0; }
  bool active() const { return _active; }
  int drive(bool tilt) const { return _active && tilt==_tilt ? (int)(_output>0 ? _output+0.5 : _output-0.5) : 0; }
  BenchDriveStatus status() const
  { BenchDriveStatus value={_count,_active,_recording,_reason}; return value; }
  bool sample(uint16_t index, BenchDriveSample &value) const
  { if(index>=_count) return false; value=_samples[index]; return true; }

private:
  bool inside(int32_t pan,int32_t tilt) const
  { return pan>=_panMin && pan<=_panMax && tilt>=_tiltMin && tilt<=_tiltMax; }
  static int64_t distance(int32_t a,int32_t b)
  { int64_t value=(int64_t)a-b; return value<0 ? -value : value; }
  void append(uint32_t now,int32_t pan,int32_t tilt)
  {
    if(_count>=192) { stop(now,"TRACE_FULL"); _recording=false; return; }
    BenchDriveSample value={uint32_t(now-_start),pan,tilt,(int16_t)drive(_tilt),
      (uint8_t)(!_active ? 3 : !_boosted ? 0 : _cruising ? 2 : 1)};
    _samples[_count++]=value;
  }
  int32_t _panMin,_panMax,_tiltMin,_tiltMax;
  BenchDriveSample _samples[192];
  uint16_t _count;
  bool _active,_recording,_tilt;
  int _requested;
  double _output;
  uint32_t _start,_last,_end,_duration;
  int32_t _travel,_panStart,_tiltStart;
  const char *_reason;
  bool _boosted, _cruising;
  int _cruisePwm;
};
#endif
