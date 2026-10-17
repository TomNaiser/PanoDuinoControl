#include "MotionConfigStore.h"
#include <string.h>
static const uint32_t MAGIC=0x4D4F5431UL;
// 2.4.36 stored the unchanged prefix of 85 settings. Its CRC occupies what
// is now values[85]. Preserve that complete, validated record on upgrade.
static const uint16_t LEGACY_PARAMETER_COUNT=85;
static_assert(MP_COLUMN_RETURN_TEMPO==LEGACY_PARAMETER_COUNT,"Legacy setting indices must remain stable");
static_assert(MP_INCREMENTAL_MOVE_MAX_DURATION_MS==86,"2.4.37 setting indices must remain stable");
static_assert(MP_PAN_TARGET_RECOVERY_I_GAIN==115,"2.4.65 setting indices must remain stable");
static_assert(MP_PAN_TARGET_PREDICTION_SAMPLES==117,"2.4.66 setting indices must remain stable");
static_assert(MP_MOVE_AUTO_DURATION==120,"2.4.68 setting indices must remain stable");
static_assert(MP_CAPTURE_PAN_WARNING_DEG==124,"2.4.69 setting indices must remain stable");
uint32_t MotionConfigStore::checksum(const StoredMotionConfig &r)
{
  uint32_t crc=0xFFFFFFFFUL;
  const uint8_t *bytes=(const uint8_t*)&r.schema;
  unsigned int count=sizeof(r.schema)+sizeof(r.count)+sizeof(r.revision)+r.count*sizeof(r.values[0]);
  for(unsigned int i=0;i<count;++i)
  {
    crc^=bytes[i];
    for(unsigned int b=0;b<8;++b) crc=(crc>>1)^((crc&1) ? 0xEDB88320UL : 0);
  }
  return ~crc;
}
bool MotionConfigStore::valid(const StoredMotionConfig &r)
{
  if(r.magic!=MAGIC || r.schema!=MotionSettings::SCHEMA ||
     (r.count<LEGACY_PARAMETER_COUNT || r.count>MOTION_PARAMETER_COUNT) || !r.revision) return false;
  uint32_t storedChecksum;
  memcpy(&storedChecksum,(const uint8_t*)r.values+r.count*sizeof(r.values[0]),sizeof(storedChecksum));
  if(storedChecksum!=checksum(r)) return false;
  MotionSettings s; memcpy(s.values,r.values,r.count*sizeof(r.values[0])); return !s.validate();
}
bool MotionConfigStore::load(MotionSettings &settings,uint32_t &revision)
{
  StoredMotionConfig a={},b={};
  bool va=_storage.read(0,a) && valid(a), vb=_storage.read(1,b) && valid(b);
  if(!va && !vb) return false;
  const StoredMotionConfig &r=va && (!vb || (int32_t)(a.revision-b.revision)>0) ? a : b;
  for(unsigned int i=0;i<MOTION_PARAMETER_COUNT;++i)
    settings.values[i]=i<r.count ? r.values[i] : motionParameter(i).defaultValue;
  revision=r.revision; return true;
}
bool MotionConfigStore::save(const MotionSettings &settings,uint32_t revision)
{
  if(settings.validate() || !revision) return false;
  StoredMotionConfig a={},b={};
  bool va=_storage.read(0,a) && valid(a), vb=_storage.read(1,b) && valid(b);
  uint8_t slot=va && (!vb || (int32_t)(a.revision-b.revision)>0) ? 1 : 0;
  StoredMotionConfig r={}; r.magic=MAGIC; r.schema=MotionSettings::SCHEMA;
  r.count=MOTION_PARAMETER_COUNT; r.revision=revision;
  memcpy(r.values,settings.values,sizeof(r.values)); r.checksum=checksum(r);
  if(!_storage.write(slot,r)) return false;
  StoredMotionConfig verified={};
  return _storage.read(slot,verified) && valid(verified) && !memcmp(&r,&verified,sizeof(r));
}
