#ifndef PANODUINO_MOTION_CONFIG_STORE_H
#define PANODUINO_MOTION_CONFIG_STORE_H
#include "MotionSettings.h"

struct StoredMotionConfig
{
  uint32_t magic;
  uint16_t schema, count;
  uint32_t revision;
  int32_t values[MOTION_PARAMETER_COUNT];
  uint32_t checksum;
};
// Nano R4 slots begin at 1024 and 2048 (1024 bytes apart).
// Preserve the addresses and leave 256 bytes spare; old prefix CRCs still load.
static_assert(sizeof(StoredMotionConfig)<=768,"Motion EEPROM slot exceeds reserved interval");

class IMotionConfigStorage
{
public:
  virtual ~IMotionConfigStorage() {}
  virtual bool read(uint8_t slot,StoredMotionConfig &record)=0;
  virtual bool write(uint8_t slot,const StoredMotionConfig &record)=0;
};

// Two complete slots; a failed write leaves the preceding revision readable.
// Storage is injected so torn writes and corrupt records exercise this path.
class MotionConfigStore
{
public:
  explicit MotionConfigStore(IMotionConfigStorage &storage):_storage(storage) {}
  bool load(MotionSettings &settings,uint32_t &revision);
  bool save(const MotionSettings &settings,uint32_t revision);
private:
  static uint32_t checksum(const StoredMotionConfig &record);
  static bool valid(const StoredMotionConfig &record);
  IMotionConfigStorage &_storage;
};
#endif
