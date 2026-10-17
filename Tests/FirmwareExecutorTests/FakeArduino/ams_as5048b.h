#ifndef TEST_AMS_AS5048B_H
#define TEST_AMS_AS5048B_H

class AMS_AS5048B
{
public:
  explicit AMS_AS5048B(int) {}
  void begin() {}
  void setClockWise(bool) {}
  void zeroRegW(int) {}
  void setZeroReg() {}
  void updateMovingAvgExp() {}
  double angleR(int, bool) { return 0.0; }
};

#endif
