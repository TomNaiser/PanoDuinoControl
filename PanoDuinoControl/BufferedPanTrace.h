#ifndef PANODUINO_BUFFERED_PAN_TRACE_H
#define PANODUINO_BUFFERED_PAN_TRACE_H
#include <stdint.h>

// Diagnostic build only. No serial IO in sample(): one eight-byte record per
// control tick, frozen when drive is disabled. Reading never changes control.
class BufferedPanTrace
{
public:
  struct Sample { int32_t positionMdeg; int16_t pwm; uint16_t deltaMs; };
  static const unsigned int CAPACITY=768;
  BufferedPanTrace():_head(0),_count(0),_overwritten(0),_invalid(0),_lastMs(0),
    _previousMs(0),_previousPosition(0),_recording(false),_ready(false),_previous(false),_badTiming(false) {}
  void sample(uint32_t now,int32_t position,int16_t pwm,bool enabled,bool valid)
  {
    if(!_recording && pwm!=0) {
      _head=_count=0;_overwritten=_invalid=0;_ready=false;_badTiming=false;_recording=true;
      _lastMs=now;
      if(_previous) { append(_previousPosition,0,0);_lastMs=_previousMs; }
    }
    if(_recording) {
      uint32_t delta=uint32_t(now-_lastMs);
      if(delta>100) _badTiming=true;
      append(position,pwm,(uint16_t)(delta>65535 ? 65535 : delta));
      _lastMs=now;if(!valid) ++_invalid;
      if(!enabled && pwm==0) { _recording=false;_ready=true; }
    }
    _previousMs=now;_previousPosition=position;_previous=true;
  }
  bool ready() const { return _ready; }
  unsigned int count() const { return _count; }
  uint32_t overwritten() const { return _overwritten; }
  uint32_t invalid() const { return _invalid; }
  bool badTiming() const { return _badTiming; }
  const Sample &at(unsigned int index) const { return _samples[(_head+index)%CAPACITY]; }
  uint16_t checksum() const {
    uint16_t crc=0xFFFF;
    for(unsigned int i=0;i<_count;++i) {
      const Sample &s=at(i);uint32_t position=(uint32_t)s.positionMdeg;
      for(unsigned int b=0;b<4;++b) addByte(crc,(uint8_t)(position>>(b*8)));
      addByte(crc,(uint8_t)s.pwm);addByte(crc,(uint8_t)((uint16_t)s.pwm>>8));
      addByte(crc,(uint8_t)s.deltaMs);addByte(crc,(uint8_t)(s.deltaMs>>8));
    }
    return crc;
  }
private:
  static void addByte(uint16_t &crc,uint8_t byte) {
    crc^=(uint16_t)byte<<8;
    for(unsigned int b=0;b<8;++b) crc=(uint16_t)((crc&0x8000) ? (crc<<1)^0x1021 : crc<<1);
  }
  void append(int32_t position,int16_t pwm,uint16_t delta) {
    unsigned int index=(_head+_count)%CAPACITY;
    if(_count==CAPACITY) { _head=(_head+1)%CAPACITY;++_overwritten; }
    else ++_count;
    _samples[index].positionMdeg=position;_samples[index].pwm=pwm;_samples[index].deltaMs=delta;
  }
  Sample _samples[CAPACITY];
  unsigned int _head,_count;
  uint32_t _overwritten,_invalid,_lastMs,_previousMs;
  int32_t _previousPosition;
  bool _recording,_ready,_previous,_badTiming;
};
static_assert(sizeof(BufferedPanTrace::Sample)==8,"Buffered Pan trace record must stay compact");
#endif
