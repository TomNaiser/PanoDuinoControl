#include "MotionSettings.h"

const char *MotionSettings::validate() const
{
  for(unsigned int i=0;i<MOTION_PARAMETER_COUNT;++i)
    if(values[i]<motionParameter(i).minimum || values[i]>motionParameter(i).maximum)
      return "CONFIG_VALUE_RANGE";
  if(values[MP_PAN_TARGET_PREDICTION_SAMPLES]!=0 && values[MP_PAN_TARGET_PREDICTION_SAMPLES]<5)
    return "CONFIG_MODEL_WINDOW";
  if(values[MP_MOVE_AUTO_DURATION] && values[MP_MOVE_AUTO_MAX_MS]<values[MP_INCREMENTAL_MOVE_MAX_DURATION_MS])
    return "CONFIG_DURATION_LIMIT";
  if(number(MP_POSITION_EXIT_DEG)<=number(MP_POSITION_ENTER_DEG) ||
     number(MP_TRAVEL_POSITION_EXIT_DEG)<=number(MP_TRAVEL_POSITION_ENTER_DEG) ||
     number(MP_VELOCITY_EXIT_DPS)<=number(MP_VELOCITY_ENTER_DPS)) return "CONFIG_HYSTERESIS";
  if(number(MP_FINE_STOP_ERROR_DEG)>=number(MP_TRAVEL_POSITION_ENTER_DEG) ||
     number(MP_FINE_MAX_DISTANCE)<=number(MP_TRAVEL_POSITION_ENTER_DEG) ||
     (values[MP_FINE_ENABLED] && values[MP_FINE_MAX_PASSES]!=1)) return "CONFIG_FINE_WINDOW";
  if(number(MP_FINE_MAX_PWM)>number(MP_TRAVEL_PWM_MAX) ||
     number(MP_FINE_PWM_SLEW)>number(MP_PWM_SLEW_PER_SECOND) ||
     number(MP_TRAVEL_DOWN_START_PWM_MAX)>number(MP_TRAVEL_PWM_MAX)) return "CONFIG_POWER_LIMIT";
  for(unsigned int a=0;a<2;++a)
  {
    AxisMotionConfiguration c=axis(a!=0);
    if(number(a ? MP_RUN_TILT_MAX_VELOCITY : MP_RUN_PAN_MAX_VELOCITY)<c.maximumVelocity) return "CONFIG_RUN_SPEED";
    if((c.positiveRunningPwm==0)!=(c.negativeRunningPwm==0)) return "CONFIG_RUNNING_PAIR";
    if(c.frictionFeedforwardPwm>c.maximumPwm || c.negativeFrictionFeedforwardPwm>c.maximumPwm ||
       c.positiveRunningPwm>c.maximumPwm || c.negativeRunningPwm>c.maximumPwm) return "CONFIG_AXIS_POWER";
    if(a && (number(MP_TRAVEL_PWM_MAX)>c.maximumPwm ||
      c.positiveRunningPwm>number(MP_TRAVEL_PWM_MAX) || c.negativeRunningPwm>number(MP_TRAVEL_PWM_MAX) ||
      (c.positiveRunningPwm>0 && number(MP_TRAVEL_PWM_REDUCTION_MAX)>=c.positiveRunningPwm) ||
      (c.negativeRunningPwm>0 && number(MP_TRAVEL_DOWN_PWM_REDUCTION_MAX)>=c.negativeRunningPwm))) return "CONFIG_TRAVEL_POWER";
  }
  if(values[MP_FINE_ENABLED] && (axis(true).positiveRunningPwm<=0 ||
      number(MP_FINE_MAX_VELOCITY)>axis(true).maximumVelocity ||
      number(MP_FINE_MAX_ACCELERATION)>axis(true).maximumAcceleration ||
      number(MP_FINE_MAX_JERK)>axis(true).maximumJerk)) return "CONFIG_FINE_AXIS";
  if(axis(true).positiveRunningPwm>0 && (number(MP_JOG_MIN_PWM)>axis(true).positiveRunningPwm ||
      number(MP_JOG_MIN_PWM)>axis(true).negativeRunningPwm)) return "CONFIG_JOG_POWER";
  if(number(MP_PAN_JOG_START_PWM)>axis(false).maximumPwm ||
      number(MP_PAN_JOG_RUNNING_PWM)>number(MP_PAN_JOG_START_PWM) ||
      number(MP_JOG_MIN_PWM)>number(MP_PAN_JOG_RUNNING_PWM)) return "CONFIG_JOG_POWER";
  return 0;
}

AxisMotionConfiguration MotionSettings::axis(bool tilt) const
{
  unsigned int start=tilt ? MP_TILT_MAXIMUM_VELOCITY : MP_PAN_MAXIMUM_VELOCITY;
  AxisMotionConfiguration c={
    number((MotionParameterId)(start+0)),number((MotionParameterId)(start+1)),
    number((MotionParameterId)(start+2)),number((MotionParameterId)(start+3)),
    number((MotionParameterId)(start+4)),number((MotionParameterId)(start+5)),
    number((MotionParameterId)(start+6)),number((MotionParameterId)(start+7)),
    number((MotionParameterId)(start+8)),number((MotionParameterId)(start+9)),
    number((MotionParameterId)(start+10)),number((MotionParameterId)(start+11)),
    number((MotionParameterId)(start+12)),number((MotionParameterId)(start+13))};
  return c;
}

void MotionSettings::setAxis(bool tilt,const AxisMotionConfiguration &c)
{
  unsigned int start=tilt ? MP_TILT_MAXIMUM_VELOCITY : MP_PAN_MAXIMUM_VELOCITY;
  const double fields[]={c.maximumVelocity,c.maximumAcceleration,c.maximumJerk,c.proportionalGain,
    c.velocityGain,c.maximumPwm,c.frictionFeedforwardPwm,c.velocityFeedforwardGain,
    c.negativeFrictionFeedforwardPwm,c.velocityIntegralGain,c.positionVelocityGain,
    c.positiveRunningPwm,c.negativeRunningPwm,c.trackingIntegralGain};
  for(unsigned int i=0;i<14;++i) values[start+i]=(int32_t)lround(fields[i]*motionParameter(start+i).scale);
}
