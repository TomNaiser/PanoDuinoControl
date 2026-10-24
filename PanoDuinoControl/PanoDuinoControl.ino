/* PanoDuinoControl — profiled V2 motion on the existing PanoBoard V5.
   Author: Thomas Naiser. Legacy PC firmware is retained in fallback/.
   Positive tilt remains up; motor polarity is applied only at the pins. */
#include <ams_as5048b.h>
#include <Wire.h>
#include <math.h>
#include "V2Controller.h"
#include "MotionController.h"
#include "SerialTransport.h"
#include "VersionInfo.h"
#include "MotionConfigStore.h"
#if defined(PANODUINO_BUFFERED_PAN_TRACE)
#if !defined(ARDUINO_NANO_R4) || !defined(PANODUINO_MOTION_TRACE)
#error Buffered Pan tracing requires the explicit Nano R4 Bluetooth trace build.
#endif
#include "BufferedPanTrace.h"
#include <string.h>
static BufferedPanTrace bufferedPanTrace;
#endif
#if defined(ARDUINO_NANO_R4)
#include <EEPROM.h>
#endif
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_MOTION_TRACE)
#include <stdio.h>
#if defined(PANODUINO_USB_CONTROL)
#error Motion tracing requires normal Bluetooth control, not USB bench control.
#endif
#endif
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
#include "FspTimer.h"
extern FspTimer* __get_timer_for_channel(int channel);
#endif

const uint8_t PanSensorADDRESS = 0x41, TiltSensorADDRESS = 0x40;
const int pwm_x = 3, pwm_y = 9;
const int dir_x = 2, dir_x_neg = 4, dir_y = 11, dir_y_neg = 12;
const int shutterPinLumix1 = 7, shutterPinLumix2 = 8, shutterPinSony = 5, focusPinSony = 6;
bool PanAxisSensorAvailable = false, TiltAxisSensorAvailable = false;
double AbsPos_x = 0, AbsPos_y = 0, TargetPos_x = 0, TargetPos_y = 0;
double PreviousPosition_x = 0, PreviousPosition_y = 0, TotalOffset_x = 0, TotalOffset_y = 0;
int lastPanDrive = 0, lastTiltDrive = 0;
#if defined(PANODUINO_MOTION_TRACE) || defined(PANODUINO_HOST_TEST)
uint8_t encoderReadError[2] = {0,0};
uint32_t encoderReadFailures[2] = {0,0};
#endif
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
uint32_t encoderBusRecoveries = 0;
static uint8_t encoderRecoveryBurst = 0;
static uint32_t encoderLastRecoveryMs = 0, encoderHealthySinceMs = 0, encoderReadStartUs = 0;
static bool encoderHealthyClock = false, encoderReadBudgetActive = false;
#endif

static bool encoderReadResult(uint8_t address, uint8_t error)
{
#if defined(PANODUINO_MOTION_TRACE) || defined(PANODUINO_HOST_TEST)
  unsigned int axis = address == PanSensorADDRESS ? 0 : 1;
  encoderReadError[axis] = error;
  if(error) ++encoderReadFailures[axis];
#else
  (void)address;
#endif
  return error == 0;
}

#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
auto &Wireless_Serial = Serial; // Explicit bench build; same V2 executor and motion controller.
#elif defined(ARDUINO_NANO_R4)
auto &Wireless_Serial = Serial1;
#elif defined(DEBUG)
auto &Wireless_Serial = Serial2;
#else
auto &Wireless_Serial = Serial;
#endif
#if defined(ARDUINO_NANO_R4) && !defined(PANODUINO_USB_CONTROL)
SerialTransport wirelessTransport(Wireless_Serial, true);
#else
SerialTransport wirelessTransport(Wireless_Serial);
#endif

#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
class BoardMotionConfigStorage : public IMotionConfigStorage
{
public:
  bool read(uint8_t slot,StoredMotionConfig &record)
  {
#if defined(ARDUINO_NANO_R4)
    int address=slot ? 2048 : 1024;
    if(address+(int)sizeof(record)>EEPROM.length()) return false;
    EEPROM.get(address,record);
#else
    record=records[slot];
#endif
    return true;
  }
  bool write(uint8_t slot,const StoredMotionConfig &record)
  {
#if defined(ARDUINO_NANO_R4)
    int address=slot ? 2048 : 1024;
    if(address+(int)sizeof(record)>EEPROM.length()) return false;
    EEPROM.put(address,record);
#else
    records[slot]=record;
#endif
    return true;
  }
#if defined(PANODUINO_HOST_TEST)
  StoredMotionConfig records[2]={};
#endif
};
static BoardMotionConfigStorage boardMotionConfigStorage;
#endif

class PanoHardwareAdapter : public IPanoramaHardware
{
public:
  PanoHardwareAdapter() : _tiltZeroOffsetMdeg(0) {}
  uint16_t columnReturnTiltTempo(uint16_t rowTempo) const
  {
    uint16_t configured=(uint16_t)_motion.settings().values[MP_COLUMN_RETURN_TEMPO];
    return configured ? configured : rowTempo;
  }
  uint32_t incrementalMoveMaximumDurationMs() const
  { return _motion.plannedMoveMaximumDurationMs(); }
  uint32_t positioningMoveMaximumDurationMs() const
  { return _motion.settings().values[MP_MOVE_AUTO_DURATION] ? _motion.plannedMoveMaximumDurationMs() : 0; }
  bool supportsManualJog() const { return true; }
  bool startManualJog(bool tilt,int direction,uint16_t tempo)
  { return PanAxisSensorAvailable && TiltAxisSensorAvailable && _motion.commandJog(tilt,direction,tempo); }
  uint16_t manualJogMaximumVelocityMdeg() const { return (uint16_t)_motion.settings().values[MP_JOG_MAX_VELOCITY]; }
  uint16_t manualJogMaximumPanVelocityMdeg() const { return (uint16_t)_motion.settings().values[MP_PAN_JOG_MAX_VELOCITY]; }
  bool startManualJogVelocity(bool tilt,int direction,uint16_t velocityMdeg)
  { return PanAxisSensorAvailable && TiltAxisSensorAvailable && _motion.commandJogVelocity(tilt,direction,velocityMdeg/1000.0); }
  bool manualJogLimitReached() const { return _motion.jogLimitReached(); }
  uint32_t manualJogLeaseMs() const { return _motion.settings().milliseconds(MP_JOG_LEASE_MS); }
  bool setTargetMdeg(int32_t pan, int32_t tilt, uint16_t panSpeed, uint16_t tiltSpeed, bool)
  {
    if (!PanAxisSensorAvailable || !TiltAxisSensorAvailable ||
        !_motion.commandPositioning(pan / 1000.0, tilt / 1000.0, panSpeed, tiltSpeed)) return false;
    TargetPos_x = pan / 1000.0;
    TargetPos_y = ((double)tilt + _tiltZeroOffsetMdeg) / 1000.0;
    return true;
  }
  bool setManualTargetMdeg(int32_t pan,int32_t tilt,uint16_t panSpeed,uint16_t tiltSpeed)
  {
    if(!PanAxisSensorAvailable || !TiltAxisSensorAvailable ||
       !_motion.commandManual(pan/1000.0,tilt/1000.0,panSpeed,tiltSpeed)) return false;
    TargetPos_x=pan/1000.0; TargetPos_y=((double)tilt+_tiltZeroOffsetMdeg)/1000.0; return true;
  }
  bool targetsReached() { return _motion.reached(); }
  bool setColumnReturnTargetMdeg(int32_t pan,int32_t tilt,uint16_t panSpeed,uint16_t tiltSpeed)
  {
    if(!PanAxisSensorAvailable || !TiltAxisSensorAvailable ||
       !_motion.commandColumnReturn(pan/1000.0,tilt/1000.0,panSpeed,tiltSpeed)) return false;
    TargetPos_x=pan/1000.0; TargetPos_y=((double)tilt+_tiltZeroOffsetMdeg)/1000.0; return true;
  }
  bool setPositioningTargetMdeg(int32_t pan,int32_t tilt,uint16_t panSpeed,uint16_t tiltSpeed)
  {
    if(!PanAxisSensorAvailable || !TiltAxisSensorAvailable ||
       !_motion.commandPositioning(pan/1000.0,tilt/1000.0,panSpeed,tiltSpeed)) return false;
    TargetPos_x=pan/1000.0; TargetPos_y=((double)tilt+_tiltZeroOffsetMdeg)/1000.0; return true;
  }
  bool exposurePositionStable(int32_t pan,int32_t tilt)
  { return PanAxisSensorAvailable && TiltAxisSensorAvailable && _motion.passiveCaptureStable(millis(),pan/1000.0,tilt/1000.0); }
  uint32_t captureStabilityWaitMs() const { return _motion.settings().milliseconds(MP_CAPTURE_SETTLE_WAIT_MS); }
  bool stoppedPositionResting() const
  { return PanAxisSensorAvailable && TiltAxisSensorAvailable && _motion.feedbackResting(millis()); }
  bool stoppedPositionStable(int32_t pan,int32_t tilt) const
  {
    return stoppedPositionResting() &&
      fabs(double(actualPanMdeg())-pan)<=1000.0*_motion.settings().number(MP_POSITION_EXIT_DEG) &&
      fabs(double(actualTiltMdeg())-tilt)<=1000.0*_motion.settings().number(MP_TRAVEL_POSITION_EXIT_DEG);
  }

  bool targetQuietSince(uint32_t &since) const { return _motion.quietSince(since); }
  const char *motionFault() const { return _motion.fault(); }
  void configureMotionLimits(int32_t panMin, int32_t panMax, int32_t tiltMin, int32_t tiltMax)
  {
    _motion.configureLimits(panMin / 1000.0, panMax / 1000.0, tiltMin / 1000.0, tiltMax / 1000.0);
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
    _bench.configureLimits(panMin,panMax,tiltMin,tiltMax);
#endif
  }
  void controlSampleCompleted(uint32_t now, bool enabled)
  {
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
    bool benchActive=_bench.active();
    _bench.tick(now,actualPanMdeg(),actualTiltMdeg(),PanAxisSensorAvailable && TiltAxisSensorAvailable,enabled);
    if(benchActive) enabled=false; // Preserve estimator, disable all profile torque.
#endif
    _motion.tick(now, AbsPos_x, actualTiltMdeg() / 1000.0,
      PanAxisSensorAvailable && TiltAxisSensorAvailable, enabled);
  }
  int drive(bool tilt) const
  {
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
    if(_bench.active()) return _bench.drive(tilt);
#endif
    return _motion.status(tilt).signedPwm;
  }
  void setCameraActive(bool active)
  {
    int level = active ? HIGH : LOW;
    digitalWrite(shutterPinLumix1, level); digitalWrite(shutterPinLumix2, level);
    digitalWrite(focusPinSony, level); digitalWrite(shutterPinSony, level);
  }
  void stopAllMotion()
  {
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
    _bench.stop(millis(),"STOPPED");
#endif
    _motion.disable(); analogWrite(pwm_x, 0); analogWrite(pwm_y, 0); lastPanDrive = lastTiltDrive = 0;
  }
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
  bool startBenchDrive(bool tilt,int pwm,uint32_t duration,int32_t travel,uint32_t now)
  {
    if(!_bench.start(tilt,pwm,duration,travel,now,actualPanMdeg(),actualTiltMdeg(),
        _motion.feedbackResting(now) && PanAxisSensorAvailable && TiltAxisSensorAvailable)) return false;
    _motion.disable(); setCameraActive(false); return true;
  }
  bool getBenchStatus(BenchDriveStatus &value) const { value=_bench.status(); return true; }
  bool startBenchCruise(bool tilt,int pwm,uint32_t duration,int32_t travel,uint32_t now)
  {
    if(!_bench.startCruise(tilt,pwm,duration,travel,now,actualPanMdeg(),actualTiltMdeg(),
        _motion.feedbackResting(now) && PanAxisSensorAvailable && TiltAxisSensorAvailable)) return false;
    _motion.disable(); setCameraActive(false); return true;
  }
  bool getBenchSample(uint16_t index,BenchDriveSample &value) const { return _bench.sample(index,value); }
  bool getBenchOutput(BenchOutputStatus &value) const
  {
    FspTimer *timer=__get_timer_for_channel(GET_CHANNEL(getPinCfgs(pwm_y,PIN_CFG_REQ_PWM)[0]));
    if(!timer) return false;
    value.panAppliedPwm=lastPanDrive; value.tiltAppliedPwm=lastTiltDrive;
    value.tiltPositivePin=digitalRead(dir_y); value.tiltNegativePin=digitalRead(dir_y_neg);
    value.tiltPeriod=timer->get_period_raw(); value.tiltDuty=timer->get_duty_cycle();
    value.tiltCounter=timer->get_counter();
    value.tiltPeripheralMux=(R_PFS->PORT[g_pin_cfg[pwm_y].pin >> IOPORT_PRV_PORT_OFFSET].PIN[g_pin_cfg[pwm_y].pin & BSP_IO_PRV_8BIT_MASK].PmnPFS & IOPORT_CFG_PERIPHERAL_PIN)!=0;
    return true;
  }
#endif
  int32_t actualPanMdeg() const { return (int32_t)lround(AbsPos_x * 1000.0); }
  int32_t actualTiltMdeg() const { return (int32_t)lround(AbsPos_y * 1000.0) - _tiltZeroOffsetMdeg; }
  bool setTiltZeroOffsetMdeg(int32_t offset)
  {
    if (!isfinite(AbsPos_y)) return false;
    stopAllMotion(); _tiltZeroOffsetMdeg = offset;
    TargetPos_x = AbsPos_x; TargetPos_y = AbsPos_y;
    _motion.resetFeedback(); // Changed coordinates require fresh feedback; retain configured limits.
    return true;
  }
  void getMotionDiagnostics(MotionDiagnosticsV2 &diagnostics) const
  {
    AxisMotionStatus pan = _motion.status(false), tilt = _motion.status(true);
    diagnostics.sampleSequence = _motion.sampleSequence();
    diagnostics.stableSamples = _motion.confirmedSamples();
    diagnostics.requiredStableSamples = (uint8_t)((_motion.settings().milliseconds(MP_CONFIRM_MS) + MOTION_CONTROL_PERIOD_MS - 1) / MOTION_CONTROL_PERIOD_MS);
    diagnostics.panWithinTolerance = pan.withinTolerance;
    diagnostics.tiltWithinTolerance = tilt.withinTolerance;
    diagnostics.panTargetMdeg = (int32_t)lround(TargetPos_x * 1000.0);
    diagnostics.tiltTargetMdeg = (int32_t)lround(TargetPos_y * 1000.0) - _tiltZeroOffsetMdeg;
    diagnostics.panErrorMdeg = diagnostics.panTargetMdeg - actualPanMdeg();
    diagnostics.tiltErrorMdeg = diagnostics.tiltTargetMdeg - actualTiltMdeg();
  }
  uint8_t captureTargetErrorFlags(int32_t pan,int32_t tilt) const
  {
      return (fabs(double(actualPanMdeg())-pan)>1000.0*_motion.settings().number(MP_CAPTURE_PAN_WARNING_DEG) ? 1 : 0) |
        (fabs(double(actualTiltMdeg())-tilt)>1000.0*_motion.settings().number(MP_CAPTURE_TILT_WARNING_DEG) ? 2 : 0);
  }
  bool getMotionTelemetry(MotionTelemetryV2 &telemetry) const
  {
    AxisMotionStatus pan = _motion.status(false), tilt = _motion.status(true);
    telemetry.panReferenceMdeg = (int32_t)lround(pan.reference * 1000.0);
    telemetry.tiltReferenceMdeg = (int32_t)lround(tilt.reference * 1000.0);
    telemetry.panVelocityMdegPerSecond = (int32_t)lround(pan.velocity * 1000.0);
    telemetry.tiltVelocityMdegPerSecond = (int32_t)lround(tilt.velocity * 1000.0);
    telemetry.panReferenceVelocity = (int32_t)lround(pan.referenceVelocity * 1000.0);
    telemetry.tiltReferenceVelocity = (int32_t)lround(tilt.referenceVelocity * 1000.0);
    telemetry.panSignedPwm = pan.signedPwm; telemetry.tiltSignedPwm = tilt.signedPwm;
    telemetry.panPhase = (uint8_t)pan.phase; telemetry.tiltPhase = (uint8_t)tilt.phase;
    telemetry.panDriveStage=(uint8_t)pan.driveStage; telemetry.tiltDriveStage=(uint8_t)tilt.driveStage;
    return true;
  }
  void loadMotionConfiguration()
  {
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
    MotionSettings settings; uint32_t revision=0;
    MotionConfigStore store(boardMotionConfigStorage);
    if(store.load(settings,revision) && _motion.configure(settings)) _motionConfigRevision=revision;
#endif
  }
  bool getMotionConfiguration(MotionSettings &settings,uint32_t &revision) const
  {
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
    settings=_motion.settings(); revision=_motionConfigRevision; return true;
#else
    (void)settings; (void)revision; return false;
#endif
  }
  const char *applyMotionConfiguration(const MotionSettings &settings,uint32_t revision)
  {
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
    const char *error=settings.validate(); if(error) return error;
    if(lastPanDrive || lastTiltDrive || !PanAxisSensorAvailable || !TiltAxisSensorAvailable ||
       !_motion.feedbackResting(millis())) return "CONFIG_NOT_RESTING";
    if(!_motion.configurationAllowed()) return "CONFIG_BUSY";
    MotionConfigStore store(boardMotionConfigStorage);
    if(!store.save(settings,revision)) return "CONFIG_STORE_FAILED";
    if(!_motion.configure(settings)) return "CONFIG_BUSY";
    _motionConfigRevision=revision; return 0;
#else
    (void)settings; (void)revision; return "CONFIG_UNSUPPORTED";
#endif
  }
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_MOTION_TRACE)
  void writeMotionTrace(uint32_t nowMs, int appliedPanPwm, int appliedTiltPwm, bool enabled, bool sensorsValid) const
  {
#if defined(PANODUINO_BUFFERED_PAN_TRACE)
    (void)appliedTiltPwm;
    bufferedPanTrace.sample(nowMs,actualPanMdeg(),(int16_t)appliedPanPwm,enabled,sensorsValid);
#else
    // Read-only USB sideband. Drop instead of waiting for the USB owner;
    // Bluetooth control, limits and the independent stop paths are unchanged.
    static uint32_t traceSequence=0, dropped=0;
    MotionTraceSnapshot sample=_motion.traceSnapshot(true);
    MotionTraceSnapshot pan=_motion.traceSnapshot(false);
    ++traceSequence; // Invalid sensor cycles do not advance the controller's sequence.
    if(!Serial) return;
    char line[384];
    int length=snprintf(line,sizeof(line),
      "[DEBUG-MT3],%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%u,%u,%u,%lu,"
      "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%u,%ld,%lu,%lu,%u,"
      "%u,%u,%u,%lu,%lu,%lu,%lu,%u,%u\n",
      (unsigned long)traceSequence,(unsigned long)nowMs,
      (long)lround(sample.axis.position*1000.0),(long)lround(sample.axis.target*1000.0),
      (long)lround(sample.axis.velocity*1000.0),(long)lround(sample.shortVelocity*1000.0),
      (long)lround(sample.quietVelocity*1000.0),(long)lround(sample.integral*1000.0),
      sample.axis.signedPwm,appliedTiltPwm,(unsigned int)sample.axis.driveStage,
      (unsigned int)enabled,(unsigned int)sensorsValid,(unsigned long)dropped,
      (long)lround(pan.axis.position*1000.0),(long)lround(pan.axis.target*1000.0),
      (long)lround(pan.axis.reference*1000.0),(long)lround(pan.axis.velocity*1000.0),
      (long)lround(pan.axis.referenceVelocity*1000.0),(long)lround(pan.shortVelocity*1000.0),
      (long)lround(pan.quietVelocity*1000.0),(long)lround(pan.integral*1000.0),
      pan.axis.signedPwm,appliedPanPwm,(unsigned int)pan.axis.phase,(long)lround(pan.meanPosition*1000.0),
      (unsigned long)lround(pan.profileTime*1000.0),(unsigned long)pan.progressAgeMs,(unsigned int)pan.reversing,
      (unsigned int)((PanAxisSensorAvailable ? 1 : 0)|(TiltAxisSensorAvailable ? 2 : 0)),
      (unsigned int)encoderReadError[0],(unsigned int)encoderReadError[1],
      (unsigned long)encoderReadFailures[0],(unsigned long)encoderReadFailures[1],
      (unsigned long)sample.sequence,(unsigned long)sample.sampleMs,
      (unsigned int)((digitalRead(SDA)==HIGH ? 1 : 0)|(digitalRead(SCL)==HIGH ? 2 : 0)),
      (unsigned int)encoderBusRecoveries);
    if(length<=0 || length>=(int)sizeof(line) || Serial.availableForWrite()<length)
    { ++dropped; return; }
    if(Serial.write((const uint8_t*)line,(size_t)length)!=(size_t)length) ++dropped;
#endif
  }
#if defined(PANODUINO_BUFFERED_PAN_TRACE)
  void serviceBufferedPanTrace(bool enabled) const
  {
    static char request[16]={0};static uint8_t used=0;
    static bool sending=false;static unsigned int index=0;static uint16_t crc=0;
    if(!Serial) return;
    unsigned int budget=16;
    while(Serial.available()>0 && budget--) {
      int c=Serial.read();if(c=='\r') continue;
      if(c=='\n') {
        request[used]=0;
        if(!enabled && lastPanDrive==0 && lastTiltDrive==0 && bufferedPanTrace.ready() && strcmp(request,"DUMP_PAN")==0) {
          sending=true;index=0;crc=bufferedPanTrace.checksum();
        }
        used=0;
      } else if(c>=32 && c<=126 && used<sizeof(request)-1) request[used++]=(char)c;
      else used=0;
    }
    if(enabled || lastPanDrive!=0 || lastTiltDrive!=0) { sending=false;return; }
    if(!sending) return;
    char line[128];int length;
    if(index==0) length=snprintf(line,sizeof(line),"[PANBUF],1,%u,%lu,%u,%lu,%ld,%ld,%lu,%04X\n",
      bufferedPanTrace.count(),(unsigned long)bufferedPanTrace.overwritten(),(unsigned int)bufferedPanTrace.badTiming(),
      (unsigned long)bufferedPanTrace.invalid(),(long)lround(TargetPos_x*1000.0),(long)actualTiltMdeg(),
      (unsigned long)_motionConfigRevision,(unsigned int)crc);
    else if(index<=bufferedPanTrace.count()) {
      const BufferedPanTrace::Sample &s=bufferedPanTrace.at(index-1);
      length=snprintf(line,sizeof(line),"[PANSAMPLE],%u,%u,%ld,%d\n",index-1,(unsigned int)s.deltaMs,(long)s.positionMdeg,(int)s.pwm);
    } else length=snprintf(line,sizeof(line),"[PANEND],%u,%04X\n",bufferedPanTrace.count(),(unsigned int)crc);
    if(length<=0 || length>=(int)sizeof(line) || Serial.availableForWrite()<length) return;
    if(Serial.write((const uint8_t*)line,(size_t)length)!=(size_t)length) { sending=false;return; }
    if(index>bufferedPanTrace.count()) sending=false;
    else ++index;
  }
#endif
#endif
private:
  MotionController _motion;
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
  BenchDrive _bench;
#endif
  int32_t _tiltZeroOffsetMdeg;
  uint32_t _motionConfigRevision=0;
};

PanoHardwareAdapter v2Hardware;
V2Controller v2Controller(wirelessTransport, v2Hardware);
AMS_AS5048B mysensor_x(PanSensorADDRESS), mysensor_y(TiltSensorADDRESS);
#if defined(PANODUINO_HOST_TEST)
IPanoramaHardware &testHardwareAdapter() { return v2Hardware; }
void testHardwareSample(uint32_t now,bool enabled) { v2Hardware.controlSampleCompleted(now,enabled); }
#endif

void setup()
{
  delay(5000);
  Wireless_Serial.begin(57600);
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_MOTION_TRACE)
  Serial.begin(115200); // Output only; no USB movement or camera commands.
#endif
  pinMode(13, OUTPUT);
  pinMode(shutterPinLumix1, OUTPUT); pinMode(shutterPinLumix2, OUTPUT);
  pinMode(shutterPinSony, OUTPUT); pinMode(focusPinSony, OUTPUT);
  v2Hardware.setCameraActive(false);
  pinMode(pwm_x, OUTPUT); pinMode(pwm_y, OUTPUT);
  pinMode(dir_x, OUTPUT); pinMode(dir_x_neg, OUTPUT);
  pinMode(dir_y, OUTPUT); pinMode(dir_y_neg, OUTPUT);
  analogWrite(pwm_x, 0); analogWrite(pwm_y, 0);
  digitalWrite(dir_x, HIGH); digitalWrite(dir_x_neg, LOW);
  digitalWrite(dir_y, HIGH); digitalWrite(dir_y_neg, LOW);
  mysensor_x.begin(); mysensor_x.setClockWise(true);
  mysensor_y.begin(); mysensor_y.setClockWise(true);
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
  Wire.setWireTimeout(MOTION_ENCODER_TRANSFER_TIMEOUT_US);
#endif
  mysensor_y.zeroRegW(2396); // Retained physical encoder reference; UI zero remains separate.
  TotalOffset_x = TotalOffset_y = PreviousPosition_x = PreviousPosition_y = 0;
  v2Controller.begin(millis());
}

bool readEncoderDegrees(uint8_t address, double &degrees)
{
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
  uint32_t age=uint32_t(micros()-encoderReadStartUs);
  if(encoderReadBudgetActive && age>=MOTION_ENCODER_RECOVERY_BUDGET_US) return encoderReadResult(address,31);
  unsigned int budget=encoderReadBudgetActive ? (unsigned int)(MOTION_ENCODER_RECOVERY_BUDGET_US-age) : MOTION_ENCODER_TRANSFER_TIMEOUT_US;
  Wire.setWireTimeout(budget<MOTION_ENCODER_TRANSFER_TIMEOUT_US ? budget : MOTION_ENCODER_TRANSFER_TIMEOUT_US);
#endif
  Wire.beginTransmission(address); Wire.write(0xFE);
  uint8_t txError = Wire.endTransmission(false);
  if (txError != 0) return encoderReadResult(address,txError);
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
  age=uint32_t(micros()-encoderReadStartUs);
  if(encoderReadBudgetActive && age>=MOTION_ENCODER_RECOVERY_BUDGET_US) return encoderReadResult(address,31);
  budget=encoderReadBudgetActive ? (unsigned int)(MOTION_ENCODER_RECOVERY_BUDGET_US-age) : MOTION_ENCODER_TRANSFER_TIMEOUT_US;
  Wire.setWireTimeout(budget<MOTION_ENCODER_TRANSFER_TIMEOUT_US ? budget : MOTION_ENCODER_TRANSFER_TIMEOUT_US);
#endif
  unsigned int count = Wire.requestFrom(address, (uint8_t)2);
  if (count != 2 || Wire.available() != 2) return encoderReadResult(address,10+(uint8_t)count);
  int msb = Wire.read(), lsb = Wire.read();
  if (msb < 0 || lsb < 0) return encoderReadResult(address,20);
  uint16_t raw = ((uint16_t)msb << 6) | (lsb & 0x3F);
  degrees = (16383 - raw) * (360.0 / 16384.0);
  return encoderReadResult(address,isfinite(degrees) ? 0 : 30);
}

bool readAxis(uint8_t address, double &previous, double &offset, double &absolute)
{
  double angle;
  if (!readEncoderDegrees(address, angle)) return false; // Retain the last valid position.
  double delta = angle - previous;
  if (delta > 180) offset -= 360;
  if (delta < -180) offset += 360;
  absolute = angle + offset; previous = angle;
  return true;
}

void writeMotor(int pwmPin, int positivePin, int negativePin, int signedPwm, int &previousDrive)
{
  if (signedPwm == previousDrive) return; // Avoid redundant writes between control samples.
  if (!signedPwm) { analogWrite(pwmPin, 0); previousDrive = 0; return; }
  if (!previousDrive || previousDrive * signedPwm < 0)
  {
    // Disable the bridge before changing direction (coast, not hard brake).
    analogWrite(pwmPin, 0);
    digitalWrite(positivePin, signedPwm > 0 ? HIGH : LOW);
    digitalWrite(negativePin, signedPwm > 0 ? LOW : HIGH);
  }
  analogWrite(pwmPin, abs(signedPwm));
  previousDrive = signedPwm;
}

#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
static bool recoverEncoderBus(uint32_t now)
{
  // Never clear a bus with powered motors. Recovery preserves the existing
  // target only if fresh feedback is obtained within this sample's deadline.
  if(lastPanDrive || lastTiltDrive || encoderRecoveryBurst>=MOTION_ENCODER_RECOVERY_BURST_MAX ||
     (encoderRecoveryBurst && uint32_t(now-encoderLastRecoveryMs)<MOTION_ENCODER_RECOVERY_INTERVAL_MS)) return false;
  encoderLastRecoveryMs=now; ++encoderBusRecoveries; ++encoderRecoveryBurst;
  unsigned int before=(digitalRead(SDA)==HIGH ? 1U : 0U)|(digitalRead(SCL)==HIGH ? 2U : 0U);
  Wire.end();
  pinMode(SDA,INPUT_PULLUP); pinMode(SCL,INPUT_PULLUP);
  unsigned int pulses=0;
  for(;pulses<9 && digitalRead(SDA)==LOW && digitalRead(SCL)==HIGH;++pulses)
  {
    digitalWrite(SCL,LOW); pinMode(SCL,OUTPUT); delayMicroseconds(10);
    pinMode(SCL,INPUT_PULLUP); delayMicroseconds(10);
  }
  if(digitalRead(SCL)==HIGH)
  {
    digitalWrite(SCL,LOW); pinMode(SCL,OUTPUT);
    digitalWrite(SDA,LOW); pinMode(SDA,OUTPUT); delayMicroseconds(10);
    pinMode(SCL,INPUT_PULLUP); delayMicroseconds(10);
    pinMode(SDA,INPUT_PULLUP); delayMicroseconds(10);
  }
  unsigned int after=(digitalRead(SDA)==HIGH ? 1U : 0U)|(digitalRead(SCL)==HIGH ? 2U : 0U);
  Wire.begin(); Wire.setWireTimeout(MOTION_ENCODER_TRANSFER_TIMEOUT_US);
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_MOTION_TRACE)
  char report[96];
  int length=snprintf(report,sizeof(report),"[DEBUG-BUS],%lu,%u,%u,%u,%u\n",
    (unsigned long)now,(unsigned int)encoderBusRecoveries,before,after,pulses);
  if(Serial && length>0 && length<(int)sizeof(report) && Serial.availableForWrite()>=length)
    Serial.write((const uint8_t*)report,(size_t)length);
#else
  (void)before;
#endif
  return after==3;
}
#endif

static void sampleEncoders(uint32_t now)
{
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
  encoderReadStartUs=micros(); encoderReadBudgetActive=true;
#else
  (void)now;
#endif
  PanAxisSensorAvailable=readAxis(PanSensorADDRESS,PreviousPosition_x,TotalOffset_x,AbsPos_x);
  if(!PanAxisSensorAvailable)
  { writeMotor(pwm_x,dir_x,dir_x_neg,0,lastPanDrive); writeMotor(pwm_y,dir_y,dir_y_neg,0,lastTiltDrive); }
  TiltAxisSensorAvailable=readAxis(TiltSensorADDRESS,PreviousPosition_y,TotalOffset_y,AbsPos_y);
  if(!TiltAxisSensorAvailable)
  { writeMotor(pwm_x,dir_x,dir_x_neg,0,lastPanDrive); writeMotor(pwm_y,dir_y,dir_y_neg,0,lastTiltDrive); }
#if defined(ARDUINO_NANO_R4) || defined(PANODUINO_HOST_TEST)
  if(!PanAxisSensorAvailable || !TiltAxisSensorAvailable)
  {
    encoderHealthyClock=false;
    if(uint32_t(micros()-encoderReadStartUs)<MOTION_ENCODER_RECOVERY_BUDGET_US && recoverEncoderBus(now))
    {
      PanAxisSensorAvailable=readAxis(PanSensorADDRESS,PreviousPosition_x,TotalOffset_x,AbsPos_x);
      TiltAxisSensorAvailable=readAxis(TiltSensorADDRESS,PreviousPosition_y,TotalOffset_y,AbsPos_y);
    }
  }
  if(uint32_t(micros()-encoderReadStartUs)>=MOTION_ENCODER_RECOVERY_BUDGET_US)
    PanAxisSensorAvailable=TiltAxisSensorAvailable=false;
  encoderReadBudgetActive=false;
  if(PanAxisSensorAvailable && TiltAxisSensorAvailable)
  {
    if(!encoderHealthyClock) { encoderHealthyClock=true; encoderHealthySinceMs=now; }
    if(uint32_t(now-encoderHealthySinceMs)>=MOTION_ENCODER_RECOVERY_INTERVAL_MS) encoderRecoveryBurst=0;
  }
#endif
}

void loop()
{
  static bool clockStarted = false;
  static uint32_t lastSampleMs = 0;
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_MOTION_TRACE)
  bool sampleCompleted=false;
#endif
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
  // Bench control must stop when its USB owner closes the port/disconnects.
  bool usbConnected = Serial;
  if (v2Controller.motionEnabled() && !usbConnected)
    v2Controller.emergencyStop("USB_TEST_DISCONNECT");
  if (!usbConnected) wirelessTransport.discardPending();
#endif
  wirelessTransport.service();
  uint32_t now = millis();
  if (!clockStarted || (uint32_t)(now - lastSampleMs) >= MOTION_CONTROL_PERIOD_MS)
  {
    clockStarted = true;
    sampleEncoders(now);
    now=millis(); lastSampleMs=now;
    v2Hardware.controlSampleCompleted(now, v2Controller.motionEnabled());
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_MOTION_TRACE)
    sampleCompleted=true;
#endif
    if (!PanAxisSensorAvailable || !TiltAxisSensorAvailable)
      v2Controller.fault(!PanAxisSensorAvailable ? "FAULT_PAN_SENSOR" : "FAULT_TILT_SENSOR");
    else if (v2Hardware.motionFault()) v2Controller.fault(v2Hardware.motionFault());
    else v2Controller.observeTilt(v2Hardware.actualTiltMdeg());
  }
  v2Controller.tick(now); // Bounded RX work and queued TX; R4 UART drains in small chunks.
  if (!PanAxisSensorAvailable || !TiltAxisSensorAvailable)
    v2Controller.fault(!PanAxisSensorAvailable ? "FAULT_PAN_SENSOR" : "FAULT_TILT_SENSOR");
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_USB_CONTROL)
  if (!usbConnected) wirelessTransport.discardPending();
#endif
  if (wirelessTransport.takeOverflow()) v2Controller.fault("TX_OVERFLOW");
  int pan = v2Controller.limitDrive(false, v2Hardware.actualPanMdeg(), v2Hardware.drive(false));
  int tilt = v2Controller.limitDrive(true, v2Hardware.actualTiltMdeg(), v2Hardware.drive(true));
  writeMotor(pwm_x, dir_x, dir_x_neg, -pan, lastPanDrive); // Existing pan motor polarity.
  writeMotor(pwm_y, dir_y, dir_y_neg, tilt, lastTiltDrive); // Positive encoder tilt raises the lens.
#if defined(ARDUINO_NANO_R4) && defined(PANODUINO_MOTION_TRACE)
  if(sampleCompleted) v2Hardware.writeMotionTrace(now,-lastPanDrive,lastTiltDrive,v2Controller.motionEnabled(),
    PanAxisSensorAvailable && TiltAxisSensorAvailable);
#if defined(PANODUINO_BUFFERED_PAN_TRACE)
  v2Hardware.serviceBufferedPanTrace(v2Controller.motionEnabled());
#endif
#endif
  wirelessTransport.service();
}
