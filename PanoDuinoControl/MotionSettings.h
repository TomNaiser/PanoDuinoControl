#ifndef PANODUINO_MOTION_SETTINGS_H
#define PANODUINO_MOTION_SETTINGS_H

#include <stdint.h>
#include <math.h>
#include "MotionConfiguration.h"

enum MotionParameterId
{
#define MOTION_PARAMETER(id,name,unit,scale,value,minimum,maximum) MP_##id,
#include "MotionParameters.h"
#undef MOTION_PARAMETER
  MOTION_PARAMETER_COUNT
};

struct MotionParameterDescriptor
{
  const char *name, *unit;
  int32_t scale, defaultValue, minimum, maximum;
};

inline const MotionParameterDescriptor &motionParameter(unsigned int index)
{
  static const MotionParameterDescriptor catalog[] = {
#define MOTION_PARAMETER(id,name,unit,scale,value,minimum,maximum) {name,unit,scale,value,minimum,maximum},
#include "MotionParameters.h"
#undef MOTION_PARAMETER
  };
  return catalog[index]; // Caller validates the index against COUNT.
}

// All transport/storage numbers are fixed-point integers. No NaN, locale or
// floating-point serialization can change an applied value after readback.
struct MotionSettings
{
  static const uint16_t SCHEMA = 1;
  int32_t values[MOTION_PARAMETER_COUNT];
  MotionSettings()
  { for(unsigned int i=0;i<MOTION_PARAMETER_COUNT;++i) values[i]=motionParameter(i).defaultValue; }
  double number(MotionParameterId id) const
  { return (double)values[id]/motionParameter(id).scale; }
  uint32_t milliseconds(MotionParameterId id) const { return (uint32_t)values[id]; }
  bool set(unsigned int id,int32_t value)
  {
    if(id>=MOTION_PARAMETER_COUNT) return false;
    const MotionParameterDescriptor &p=motionParameter(id);
    if(value<p.minimum || value>p.maximum) return false;
    values[id]=value; return true;
  }
  const char *validate() const;
  AxisMotionConfiguration axis(bool tilt) const;
  void setAxis(bool tilt,const AxisMotionConfiguration &configuration);
};

#endif
