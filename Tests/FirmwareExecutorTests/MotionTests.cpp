#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "MotionController.h"
#include "SerialTransport.h"
#include "BenchDrive.h"
#include "BufferedPanTrace.h"
#include "PanStopModel.h"
#include "PanStopModelFixtures.h"

static int motionChecks = 0;
static void check(bool condition, const char *message)
{
  ++motionChecks;
  if (!condition) { std::cerr << "FEHLER Motion: " << message << std::endl; std::exit(1); }
}

static void initialize(MotionController &motion, double pan = 0, double tilt = 0, uint32_t start = 0)
{
  motion.configureLimits(-180, 180, -45, 30);
  for (unsigned int i = 0; i < 10; ++i) motion.tick(start + i * 20, pan, tilt, true, false);
}

static void testMotionTraceReadOnly()
{
  MotionController motion; initialize(motion);
  check(motion.command(0,2.0,500,500), "Regelfahrt fuer bewegungsfreie Spurabfrage angenommen");
  motion.tick(200,0,0,true,true);
  AxisMotionStatus before=motion.status(true);
  MotionTraceSnapshot first=motion.traceSnapshot(true), second=motion.traceSnapshot(true);
  check(first.sequence==motion.sampleSequence() && first.sequence==second.sequence && first.sampleMs==200,
    "Spurabfrage verwendet dieselbe abgeschlossene Reglerprobe ohne neue Messung");
  check(first.axis.position==before.position && first.axis.target==before.target &&
    first.axis.signedPwm==before.signedPwm && first.axis.driveStage==before.driveStage,
    "Spurabfrage beschreibt den vorhandenen Reglerzustand");
  check(first.integral==second.integral && first.shortVelocity==second.shortVelocity &&
    first.quietVelocity==second.quietVelocity && motion.status(true).signedPwm==before.signedPwm && !motion.fault(),
    "Wiederholte Spurabfrage aendert weder Leistung, Verlauf noch Fehlerschutz");
  MotionTraceSnapshot pan=motion.traceSnapshot(false);
  check(pan.sequence==first.sequence && pan.sampleMs==first.sampleMs && pan.axis.target==0 && first.axis.target==2.0,
    "Beide Achsspurwerte stammen aus demselben Regeltakt und behalten ihre Ziele");
  check(first.meanPosition==second.meanPosition && first.profileTime==second.profileTime &&
    first.progressAgeMs==second.progressAgeMs && first.reversing==second.reversing,
    "Erweiterte Spurabfrage veraendert weder Profilzeit noch Stillstands- oder Umkehrzustand");
}

static uint32_t followReference(MotionController &motion, uint32_t now)
{
  for (unsigned int i = 0; i < 500 && !motion.reached() && !motion.fault(); ++i)
  {
    AxisMotionStatus pan = motion.status(false), tilt = motion.status(true);
    now += 20; motion.tick(now, pan.reference, tilt.reference, true, true);
  }
  check(!motion.fault() && motion.reached(), "Ein nachvollzogener Sollwert wird zeitlich bestaetigt");
  return now;
}

static void testProfileBoundsAndContinuousReplan()
{
  AxisMotionConfiguration configuration = panMotionConfiguration();
  for (int direction = -1; direction <= 1; direction += 2)
  for (uint16_t speed : {uint16_t(200), uint16_t(1000)})
  {
    MotionProfile profile;
    check(profile.plan(0, 0, 0, direction * 10.0, -180, 180, speed, configuration), "S-Kurve planbar");
    bool bounded = true;
    double previousAcceleration = 0, position = 0, velocity = 0, acceleration = 0;
    double dt = profile.duration() / 1000;
    for (int i = 0; i <= 1000; ++i)
    {
      profile.evaluate(i * dt, position, velocity, acceleration);
      bounded = bounded && position * direction >= -1e-8 && position * direction <= 10.000001 &&
        std::abs(velocity) <= configuration.maximumVelocity * speed / 1000.0 + 1e-5 &&
        std::abs(acceleration) <= configuration.maximumAcceleration + 1e-5 &&
        (i == 0 || std::abs(acceleration - previousAcceleration) / dt <= configuration.maximumJerk + 1e-4);
      previousAcceleration = acceleration;
    }
    check(bounded && std::abs(position - direction * 10) < 1e-6 && velocity == 0 && acceleration == 0,
      "Gesamtes Profil begrenzt Position, Geschwindigkeit, Beschleunigung und Ruck");
  }
  MotionProfile moving;
  check(moving.plan(0, 1.0, 0.2, 5, -10, 10, 500, configuration), "Plan aus bewegtem Zustand");
  double p, v, a; moving.evaluate(0, p, v, a);
  check(std::abs(p) < 1e-9 && std::abs(v - 1) < 1e-9 && std::abs(a - 0.2) < 1e-9,
    "Umplanung behaelt Anfangsgeschwindigkeit und Anfangsbeschleunigung");
  check(!moving.plan(0, 0, 0, 31, -45, 30, 500, configuration), "Auswaertiges Ziel abgelehnt");
  check(moving.plan(31, 0, 0, 30.5, -45, 30, 500, configuration), "Teilweise Rueckfahrt von ausserhalb erlaubt");
  check(!moving.plan(31, 0, 0, 31.1, -45, 30, 500, configuration), "Weiterfahrt nach aussen bleibt gesperrt");
}

static void testArrivalHoldingAndReplan()
{
  MotionController motion;
  check(!motion.command(1, 0, 500, 500), "Ohne frisches Feedback kein Fahrbefehl");
  initialize(motion);
  check(motion.command(1, 0, 500, 500), "Ein-Grad-Schritt angenommen");
  check(!motion.reached(), "Schritt innerhalb alter 1.5-Grad-Toleranz nicht sofort fertig");
  uint32_t now = followReference(motion, 180);
  check(motion.status(false).phase == MOTION_HOLDING, "Normale Ankunft geht in Halten");
  check(motion.status(false).signedPwm == 0 && motion.status(true).signedPwm == 0, "Ruhige Achsen werden nicht dauernd bestromt");
  bool noChatter = true;
  for (int i = 0; i < 40; ++i)
  {
    now += 20; motion.tick(now, 1 + (i % 2 == 0 ? 0.022 : -0.022), 0, true, true);
    noChatter = noChatter && motion.status(false).signedPwm == 0 && motion.reached();
  }
  check(noChatter, "Encoderrauschen erzeugt keine wechselnden Halteimpulse");
  for (int i = 1; i <= 100; ++i) { now += 20; motion.tick(now, 1 + i * 0.002, 0, true, true); }
  for (int i = 0; i < 30; ++i) { now += 20; motion.tick(now, 1.20, 0, true, true); }
  check(motion.status(false).signedPwm == 0 && motion.reached(), "Zulaessige Restabweichung bleibt ohne Gegenkorrektur stabil");
  check(motion.command(2, 0, 500, 500), "Naechster Schritt aus Halten moeglich");
  check(motion.status(true).phase == MOTION_HOLDING && motion.status(true).quiet, "Andere Achse verliert ihren Haltezustand nicht");
  for (int i = 0; i < 20; ++i)
  {
    AxisMotionStatus state = motion.status(false); now += 20;
    motion.tick(now, state.reference, 0, true, true);
  }
  AxisMotionStatus before = motion.status(false);
  check(motion.command(3, 0, 500, 500), "Ziel waehrend Bewegung neu planbar");
  AxisMotionStatus after = motion.status(false);
  check(std::abs(after.reference - before.reference) < 1e-9 &&
    std::abs(after.referenceVelocity - before.referenceVelocity) < 1e-9 &&
    std::abs(after.referenceAcceleration - before.referenceAcceleration) < 1e-9,
    "Neues Ziel verursacht keinen Sollwert- oder Geschwindigkeitssprung");
  check(!motion.command(4, 31, 500, 500) && motion.status(false).target == 3,
    "Abgelehnter Tiltwinkel aendert auch das Pan-Ziel nicht");
  motion.disable(); now += 20; motion.tick(now, 2, 0, true, true);
  check(!motion.reached() && motion.status(false).signedPwm == 0 && motion.status(false).phase == MOTION_OFF,
    "Stop laesst altes Ziel nicht automatisch wieder anlaufen");
}

static void testFaultsAndBacklashBudget()
{
  MotionController motion; initialize(motion); motion.command(5, 5, 1000, 1000);
  bool powerBounded = true;
  for (uint32_t now = 200; now < 6000 && !motion.fault(); now += 20)
  {
    motion.tick(now, 0, 0, true, true);
    powerBounded = powerBounded && std::abs(motion.status(false).signedPwm) <= MOTION_REVERSAL_PWM_MAX &&
      std::abs(motion.status(true).signedPwm) <= MOTION_REVERSAL_PWM_MAX;
  }
  check(powerBounded && motion.fault(), "Keine Abtriebsreaktion erzeugt begrenzte Leistung und endlichen Fehler");
  check(motion.status(false).signedPwm == 0 && motion.status(true).signedPwm == 0,
    "Stillstand schaltet beide Achsen ab");
  motion.tick(6000, 1, 1, true, true);
  check(motion.fault() && motion.status(false).signedPwm == 0, "Bewegter Encoder alleine quittiert keinen Fehler");

  MotionController reversal; initialize(reversal); reversal.command(1, 0, 500, 500);
  uint32_t now = followReference(reversal, 180);
  check(reversal.command(-1, 0, 500, 500), "Erforderlicher Richtungswechsel angenommen");
  bool limited = true; int previous = 0;
  for (int i = 0; i < 100 && !reversal.fault(); ++i)
  {
    now += 20; reversal.tick(now, 1, 0, true, true);
    int pwm = reversal.status(false).signedPwm;
    limited = limited && std::abs(pwm) <= MOTION_REVERSAL_PWM_MAX &&
      (reversal.fault() ? pwm == 0 : std::abs(pwm - previous) <= 6);
    previous = pwm;
  }
  check(limited && !reversal.reached(), "Umkehrspiel wird nicht durch wachsende Leistung oder Fertigmeldung uebergangen");

  MotionController fault; initialize(fault); fault.command(5, 5, 1000, 1000);
  fault.tick(200, 0, 0, false, true);
  check(fault.fault() && fault.status(false).signedPwm == 0 && fault.status(true).signedPwm == 0,
    "Ungueltige Sensoren stoppen beide Motoren");
  MotionController late; initialize(late); late.command(5, 5, 1000, 1000);
  late.tick(281, 0, 0, true, true);
  check(late.fault() && std::strcmp(late.fault(), "CONTROL_TIMEOUT") == 0,
    "Ueberalterter Regelzyklus wird nicht durch fiktive Zwischenmessungen aufgeholt");
  MotionController duplicate; initialize(duplicate); duplicate.command(1, 0, 500, 500);
  uint32_t sequence = duplicate.sampleSequence();
  duplicate.tick(180, 0, 0, true, true);
  check(duplicate.sampleSequence() == sequence && !duplicate.reached(), "Doppelter Zeitstempel bestaetigt kein Ziel");
  duplicate.tick(200, 0, 0, true, true); duplicate.tick(200, 0, 0, true, false);
  check(duplicate.status(false).phase == MOTION_OFF && duplicate.status(false).signedPwm == 0,
    "Entzug der Fahrfreigabe wirkt auch ohne neuen Zeitstempel");

  MotionController recovery; initialize(recovery, 181, 31);
  check(recovery.command(180.5, 31, 500, 500), "Rueckfahrt erlaubt auch wenn andere Achse ausserhalb gehalten wird");
  recovery.tick(200, 181, 31, true, true);
  check(recovery.status(false).signedPwm <= 0 && recovery.status(true).signedPwm == 0,
    "Am Limit bleiben nur einwaerts gerichtete Stellwerte moeglich");
}

// A deliberately simplified motor + lost motion + compliant load model.
// It verifies the state/limits logic; it is not an identified model of the mount.
struct SoftAxis
{
  double motor = 0, motorVelocity = 0, position = 0, velocity = 0;
  void step(int pwm, double dt)
  {
    motorVelocity += (pwm * 0.08 - motorVelocity) * dt / 0.12;
    motor += motorVelocity * dt;
    double contact = position;
    if (motor - position > 0.15) contact = motor - 0.15;
    if (motor - position < -0.15) contact = motor + 0.15;
    velocity += ((contact - position) * 80 - velocity * 18) * dt;
    position += velocity * dt;
  }
};

static void testSoftLoadSimulation()
{
  // This separate state/elasticity model has no dry friction and 0.08 dps/PWM.
  // Use its known running model; the default real-rig configuration is covered
  // by the noisy, direction-dependent friction regression below.
  AxisMotionConfiguration tiltConfiguration=tiltMotionConfiguration();
  tiltConfiguration.frictionFeedforwardPwm=tiltConfiguration.negativeFrictionFeedforwardPwm=0;
  tiltConfiguration.velocityFeedforwardGain=12.5;
  tiltConfiguration.positiveRunningPwm=tiltConfiguration.negativeRunningPwm=0;
  MotionController motion(panMotionConfiguration(),tiltConfiguration); initialize(motion); SoftAxis pan, tilt;
  uint32_t now = 180;
  bool safe = true, allArrived = true;
  for (double target : {1.0, 2.0, -1.0, 0.0})
  {
    bool accepted=motion.command(target,target*0.5,500,500);
    if(!accepted) std::cerr << "SoftAxis target=" << target << " position=" << tilt.position
      << " velocity=" << motion.status(true).velocity << " reference=" << motion.status(true).reference
      << " phase=" << motion.status(true).phase << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(accepted, "Simulierte Fahrt angenommen");
    for (int sample = 0; sample < 1500 && !motion.reached() && !motion.fault(); ++sample)
    {
      now += 20;
      motion.tick(now, pan.position, tilt.position, true, true);
      AxisMotionStatus p = motion.status(false), t = motion.status(true);
      safe = safe && std::isfinite(p.reference) && std::abs(p.signedPwm) <= 100 && std::abs(t.signedPwm) <= 100;
      pan.step(p.signedPwm, 0.02); tilt.step(t.signedPwm, 0.02);
    }
    if (motion.fault()) std::cerr << "Simulation: " << motion.fault() << " target=" << target << " actual=" << pan.position << std::endl;
    if(!motion.reached()) std::cerr << "SoftAxis unreached target=" << target << " pan=" << pan.position
      << " tilt=" << tilt.position << " velocity=" << motion.status(true).velocity
      << " reference=" << motion.status(true).reference << " phase=" << motion.status(true).phase << std::endl;
    allArrived = allArrived && motion.reached() && !motion.fault() &&
      std::abs(pan.position - target) <= MOTION_POSITION_EXIT_DEG && std::abs(tilt.position - target * 0.5) <= MOTION_POSITION_EXIT_DEG;
    for (int sample = 0; sample < 20; ++sample)
    {
      now += 20; motion.tick(now, pan.position, tilt.position, true, true);
      pan.step(motion.status(false).signedPwm, 0.02); tilt.step(motion.status(true).signedPwm, 0.02);
    }
  }
  check(safe && allArrived, "Weiche Modellachsen mit Totstrecke finden Ziele auch nach Richtungswechseln");
}

static void testSlowProfilesAndClockWrap()
{
  MotionController motion; initialize(motion); SoftAxis pan;
  check(motion.command(1,0,20,20), "Langsame Fahrt ist mit voller Stellreserve planbar");
  uint32_t now=180;
  bool arrived=false;
  for(int i=0;i<2000 && !motion.fault();++i)
  {
    now+=20; motion.tick(now,pan.position,0,true,true); pan.step(motion.status(false).signedPwm,0.02);
    if(motion.reached()) { arrived=true; break; }
  }
  check(arrived && !motion.fault(), "Sehr langsames Profil startet keinen Umkehrtimeout schon bei wirkungslos kleiner PWM");
  MotionController wrap; initialize(wrap,0,0,UINT32_MAX-200);
  check(wrap.command(1,0,500,500), "Ziel vor Zeitzaehlerueberlauf angenommen");
  uint32_t time=UINT32_MAX-20;
  const uint32_t jitter[4]={10,30,15,25};
  for(int i=0;i<500 && !wrap.reached() && !wrap.fault();++i)
  {
    AxisMotionStatus state=wrap.status(false); time+=jitter[i%4];
    wrap.tick(time,state.reference,0,true,true);
  }
  check(wrap.reached() && !wrap.fault(), "Zeitjitter und uint32-Ueberlauf bleiben korrekt und brauchen frische Messwerte");
}

class BackpressuredStream : public Stream
{
public:
  bool writable = false;
  int writes = 0;
  int available() { return 0; } int peek() { return -1; } int read() { return -1; }
  int availableForWrite() { return writable ? 1 : 0; }
  size_t write(unsigned char) { check(writable, "UART wird nur bei freiem Ausgang beschrieben"); ++writes; return 1; }
  size_t print(const char *) { return 0; }
};

class RenesasUartStub : public Stream
{
public:
  int writes = 0;
  int available() { return 0; } int peek() { return -1; } int read() { return -1; }
  int availableForWrite() { return 0; } // Actual core 1.6.0 inherits this default.
  size_t write(unsigned char) { ++writes; return 1; }
  size_t print(const char *) { return 0; }
};

static void testNonblockingTransport()
{
  BackpressuredStream serial; SerialTransport transport(serial);
  transport.print("A diagnostic frame"); transport.service();
  check(serial.writes == 0 && !transport.takeOverflow(), "Blockierte UART wird nicht abgewartet");
  serial.writable = true; transport.service();
  check(serial.writes == 18, "Diagnose wird nach Freigabe geordnet geleert");
  serial.writable = false;
  while (transport.availableForWrite() > 0) transport.write('x');
  check(transport.write('x') == 0 && transport.takeOverflow(), "Ueberlauf wird explizit zur Abschaltung gemeldet");
  transport.write('x'); transport.discardPending();
  serial.writable = true; int before=serial.writes; transport.service();
  check(!transport.takeOverflow() && transport.availableForWrite()>0 && serial.writes==before,
    "Geschlossener USB-Testzugang verwirft alte Meldungen und Ueberlaufstatus");
  RenesasUartStub uart; SerialTransport renesas(uart,true);
  renesas.print("A diagnostic frame"); renesas.service();
  check(uart.writes==8, "Synchronous R4 UART sends at most eight bytes despite missing buffer API");
  renesas.service(); renesas.service();
  check(uart.writes==18, "R4 UART still delivers the complete queued frame");
}

static void testManualStartWithEncoderNoise()
{
  MotionController motion;
  motion.configureLimits(-180,180,-45,30);
  // Unpowered output encoder fluctuates by four counts, as on the real rig.
  // The other axis's unchanged target must not make a small tilt step unplannable.
  for (uint32_t now=0; now<=420; now+=20)
    motion.tick(now,120.5,10.4+(now/20%2 ? -0.044 : 0.044),true,false);
  check(motion.command(120.5,10.856,200,200),
    "Ruhender Encoder mit Quantisierungsrauschen erlaubt kleinen manuellen Schritt");
  MotionController largerNoise;
  largerNoise.configureLimits(-180,180,-45,30);
  for (uint32_t now=0; now<=420; now+=20)
    largerNoise.tick(now,120.5,10.4+(now/20%2 ? -0.11 : 0.11),true,false);
  check(largerNoise.command(120.5,20.29,200,200),
    "Gemessenes groesseres Ruherauschen sperrt auch einen langsamen Zehn-Grad-Schritt nicht");
  check(std::abs(largerNoise.status(true).velocity) < MOTION_VELOCITY_EXIT_DPS,
    "Ruherauschen erzeugt keine grosse scheinbare Geschwindigkeit fuer Bremskorrekturen");
  check(largerNoise.feedbackResting(420) && !largerNoise.feedbackResting(521),
    "Messpuls verlangt frische Ruhehistorie trotz gemessenen Ruherauschens");
  MotionController moving;
  moving.configureLimits(-180,180,-45,30);
  for (uint32_t now=0; now<=400; now+=20)
    moving.tick(now,now/1000.0,0,true,false);
  check(moving.command(2,0,500,500) && std::abs(moving.status(false).referenceVelocity-1) < 1e-8,
    "Tatsaechlich bewegte Achse behaelt gemessene Anfangsgeschwindigkeit");
  check(!moving.feedbackResting(400), "Bewegte Achse erlaubt keinen Losbrech-Messpuls");
}

static void testPanRunningLoadRecovery()
{
  // Regression for the measured pattern: motion first succeeds, then a
  // larger running load stops the axis while its profile remains ahead.
  // The load numbers exercise that state; they are not a fitted motor model.
  for(int direction : {-1,1})
  {
    MotionController motion; initialize(motion);
    check(motion.command(direction*3.0,0,500,500),"Drei-Grad-Pan-Schritt mit laufendem Lastwechsel planbar");
    double position=0, velocity=0;
    int peakPwm=0;
    MotionTraceSnapshot last=motion.traceSnapshot(false);
    for(uint32_t now=200;now<15200 && !motion.fault() && !motion.reached();now+=20)
    {
      double noise=now/20%2 ? -0.022 : 0.022;
      motion.tick(now,direction*position+noise,0,true,true);
      if(!motion.fault()) last=motion.traceSnapshot(false);
      int pwm=motion.status(false).signedPwm*direction;
      peakPwm=std::max(peakPwm,std::abs(pwm));
      double friction=position<1.5 ? 20.0 : 42.0;
      double breakaway=position<1.5 ? 30.0 : 46.0;
      if(std::abs(velocity)<0.05 && std::abs(pwm)<breakaway) velocity=0;
      else
      {
        double desired=pwm>=0 ? std::max(0.0,(pwm-friction)*0.15) : std::min(0.0,(pwm+friction)*0.15);
        velocity+=(desired-velocity)*0.02/0.10;
      }
      position+=velocity*0.02;
    }
    if(motion.fault() || !motion.reached()) std::cerr << "Pan load: actual=" << position
      << " reference=" << motion.status(false).reference << " pwm=" << last.axis.signedPwm
      << " integral=" << last.integral << " reversing=" << last.reversing
      << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(!motion.fault() && motion.reached(),"Pan erreicht nach Lastwechsel das Ziel statt am eingefrorenen Profil stehenzubleiben");
    check(std::abs(position-3.0)<=MOTION_POSITION_EXIT_DEG && peakPwm<=panMotionConfiguration().maximumPwm,
      "Pan-Lastnachregelung erhaelt Zielgenauigkeit und bestehende Leistungsgrenze");
  }
}

static void testPanFinalBreakaway()
{
  for(int direction : {-1,1})
  {
    MotionController motion; initialize(motion);
    check(motion.command(direction*3.0,0,500,500),"Pan-Endannaherung mit erneuter Haftreibung planbar");
    double position=0,velocity=0;
    bool stuck=false;
    for(uint32_t now=200;now<10200 && !motion.fault() && !motion.reached();now+=20)
    {
      if(!stuck)
      {
        position=std::min(2.64,std::abs(motion.status(false).reference));
        stuck=position>=2.64;
      }
      motion.tick(now,direction*position,0,true,true);
      if(stuck)
      {
        int pwm=direction*motion.status(false).signedPwm;
        if(std::abs(velocity)<0.05 && std::abs(pwm)<44) velocity=0;
        else
        {
          double requested=pwm>=0 ? std::max(0.0,(pwm-18)*0.15) : std::min(0.0,(pwm+18)*0.15);
          velocity+=(requested-velocity)*0.02/0.10;
        }
        position+=velocity*0.02;
      }
    }
    if(motion.fault() || !motion.reached() || std::abs(position-3.0)>MOTION_POSITION_EXIT_DEG)
      std::cerr << "Pan final breakaway: dir=" << direction << " position=" << position
        << " pwm=" << motion.status(false).signedPwm << " I=" << motion.traceSnapshot(false).integral
        << " ref=" << motion.status(false).reference << " rev=" << motion.traceSnapshot(false).reversing
        << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(!motion.fault() && motion.reached() && std::abs(position-3.0)<=MOTION_POSITION_EXIT_DEG,
      "Pan baut nach erneutem Haften vor dem Ziel rechtzeitig begrenzte Anlaufleistung auf");
  }
}

static void testPanNearTargetNoise()
{
  MotionController motion; initialize(motion);
  check(motion.command(3.0,0,500,500),"Pan-Zielannaherung mit Encoder-Ausreissern planbar");
  for(uint32_t now=200;now<3200;now+=20)
    motion.tick(now,std::min(2.78,(now-200)*0.0015),0,true,true);
  // Real trace: a stalled mean about 0.22 degrees short, with individual
  // samples crossing the entry deadband. They must not reset load torque.
  const double noise[]={0.088,-0.044,-0.044};
  for(uint32_t now=3200;now<5200 && !motion.fault();now+=20)
    motion.tick(now,2.78+noise[(now/20)%3],0,true,true);
  check(!motion.fault() && !motion.reached() && motion.traceSnapshot(false).integral>4.0,
    "Einzelne Pan-Ausreisser setzen die notwendige Zielnachregelung nicht zurueck");
}

static void testStaticFrictionDrive()
{
  MotionController motion; initialize(motion);
  check(motion.command(0,1,500,500), "Schritt mit Losbrechreibung planbar");
  double position=0, velocity=0;
  for(uint32_t now=200;now<30200 && !motion.fault() && !motion.reached();now+=20)
  {
    motion.tick(now,0,position,true,true);
    int pwm=motion.status(true).signedPwm;
    double requested=std::abs(pwm)>15 ? (std::abs(pwm)-15)*0.2*(pwm>0 ? 1 : -1) : 0;
    if(std::abs(velocity)<0.02 && std::abs(pwm)<35) velocity=0;
    else velocity+=(requested-velocity)*(0.02/0.12);
    position+=velocity*0.02;
  }
  if(!motion.reached() || motion.fault()) std::cerr << "StaticFriction position=" << position
    << " velocity=" << velocity << " reference=" << motion.status(true).reference
    << " pwm=" << motion.status(true).signedPwm << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
  check(motion.reached() && !motion.fault(), "Antrieb mit hoher Losbrech- und kleinerer Gleitreibung erreicht das Ziel");
}

// A regression model for the observed drive discontinuity, not a fitted model
// of the complete rig: breakaway exceeds running torque; directions differ.
struct FrictionTiltDrive
{
  double position, velocity, runningGain, positiveFriction, negativeFriction, positiveBreakaway;
  FrictionTiltDrive(double start, double gain=0.8, double positive=13.0, double negative=21.0, double breakaway=28.0) : position(start), velocity(0), runningGain(gain), positiveFriction(positive), negativeFriction(negative), positiveBreakaway(breakaway) {}
  void step(int pwm)
  {
    double staticPwm=pwm>=0 ? positiveBreakaway : 34.0;
    if(std::abs(velocity)<0.05 && std::abs(pwm)<staticPwm) { velocity=0; return; }
    int direction=velocity==0 ? (pwm>=0 ? 1 : -1) : (velocity>0 ? 1 : -1);
    double friction=direction>0 ? positiveFriction : negativeFriction;
    double next=velocity+(pwm-direction*friction-velocity/runningGain)*0.02/0.18;
    if(next*direction<0) next=0;
    velocity=next; position+=velocity*0.02;
  }
};

static void testFineApproachThresholdsAndBudgets()
{
  // A repeatable coast undershoot: progress is real, then feedback rests at
  // target-error. This tests the decision boundary without private-state access.
  for(double error : {0.299,0.300,0.301,0.327})
  for(int direction : {-1,1})
  {
    MotionController motion; initialize(motion);
    MotionSettings settings=motion.settings(); settings.set(MP_FINE_ENABLED,1);
    check(motion.configure(settings),"Fein-Testkonfiguration im Stillstand angenommen");
    motion.command(0,direction*3.0,500,500);
    uint32_t now=180; bool fine=false;
    for(int sample=0;sample<160 && !motion.fault() && !motion.reached();++sample)
    {
      now+=20;
      double position=direction*fmin(3.0-error,sample*0.20);
      motion.tick(now,0,position,true,true);
      if(motion.status(true).driveStage==DRIVE_FINE) { fine=true; break; }
    }
    if(!(error>0.300 ? fine && !motion.fault() : !fine && motion.reached() && !motion.fault()))
      std::cerr<<"Boundary error="<<error<<" dir="<<direction<<" fine="<<fine<<" stage="<<motion.status(true).driveStage<<" reached="<<motion.reached()<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(error>0.300 ? fine && !motion.fault() : !fine && motion.reached() && !motion.fault(),
      "Feinannaeherung beginnt ausschliesslich oberhalb der Fehlertoleranz, auch exakt am Grenzwert nicht");
    if(fine)
    {
      // A blocked fine pass must end, never loop or silently accept the error.
      for(int sample=0;sample<350 && !motion.fault();++sample)
      { now+=20; motion.tick(now,0,direction*(3.0-error),true,true); }
      check(motion.fault() && motion.status(true).signedPwm==0,
        "Blockierte Feinannaeherung hat endliches Budget und schaltet ab");
    }
  }
  for(int scenario=0;scenario<4;++scenario)
  {
    MotionController motion; initialize(motion);
    MotionSettings settings=motion.settings(); settings.set(MP_FINE_ENABLED,scenario==0 ? 0 : 1);
    double error=scenario==3 ? -0.327 : scenario==2 ? 0.60 : 0.327;
    if(scenario==1) { settings.set(MP_TRAVEL_POSITION_ENTER_DEG,350); settings.set(MP_TRAVEL_POSITION_EXIT_DEG,450); }
    if(scenario==2) settings.set(MP_FINE_MAX_DISTANCE,500);
    check(motion.configure(settings),"Alternative Toleranz und begrenztes Feinfenster gueltig");
    motion.command(0,3.0,500,500); uint32_t now=180; bool noFine=true;
    for(int sample=0;sample<350 && !motion.fault() && !motion.reached();++sample)
    { now+=20; motion.tick(now,0,fmin(3.0-error,sample*.40),true,true); noFine=noFine && motion.status(true).driveStage!=DRIVE_FINE; }
    check(noFine && (scenario==1 ? motion.reached() && !motion.fault() : motion.fault() && motion.status(true).signedPwm==0),
      "Deaktiviert, unter Benutzer-Toleranz, zu weit entfernt oder ueberschossen: keine Nachkorrektur");
  }
  for(int direction : {-1,1})
  {
    MotionController motion; initialize(motion);
    MotionSettings settings=motion.settings(); settings.set(MP_FINE_ENABLED,1);
    motion.configure(settings); motion.command(0,direction*3.0,500,500);
    FrictionTiltDrive drive(direction*(3.0-0.327),direction>0 ? 1.3 : 1.6,direction>0 ? 13 : 22,direction>0 ? 21 : 15);
    uint32_t now=180; bool seen=false, limits=true, continuous=true; int starts=0, previous=0;
    MotionDriveStage previousStage=DRIVE_IDLE; double previousAcceleration=0;
    for(int sample=0;sample<1800 && !motion.fault() && !motion.reached();++sample)
    {
      now+=20; motion.tick(now,0,sample<24 ? direction*fmin(3.0-.327,sample*.20) : drive.position,true,true);
      AxisMotionStatus state=motion.status(true);
      if(state.driveStage==DRIVE_FINE && previousStage!=DRIVE_FINE) ++starts;
      if(state.driveStage==DRIVE_FINE) {
        seen=true;
        limits=limits && std::abs(state.referenceAcceleration)<=settings.number(MP_FINE_MAX_ACCELERATION)+1e-9 &&
          std::abs(state.referenceAcceleration-previousAcceleration)<=settings.number(MP_FINE_MAX_JERK)*.02+1e-9;
        previousAcceleration=state.referenceAcceleration;
        limits=limits && std::abs(state.signedPwm)<=settings.number(MP_FINE_MAX_PWM) &&
          std::abs(state.signedPwm-previous)<=std::ceil(settings.number(MP_FINE_PWM_SLEW)*0.02)+1 &&
          state.signedPwm*direction>=0;
        if(previousStage==DRIVE_FINE && previous!=0 && state.signedPwm==0) continuous=false;
      }
      previous=state.signedPwm; previousStage=state.driveStage; if(seen) drive.step(previous);
    }
    if(motion.fault() || !motion.reached()) std::cerr<<"Fine plant dir="<<direction<<" actual="<<drive.position
      <<" entries="<<starts<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(seen && starts==1 && limits && continuous,
      "Feinfahrt besteht aus einem kontinuierlichen Durchlauf mit begrenzter Leistung und PWM-Aenderung");
    check(!motion.fault() && motion.reached() && std::abs(drive.position-direction*3.0)<=0.30 && previous==0,
      "Feinfahrt erreicht im Reibungsmodell das unveraenderte Toleranzfenster");
    bool idle=true;
    for(int sample=0;sample<80;++sample) { now+=20; motion.tick(now,0,drive.position+(sample%2 ? .022 : -.022),true,true); idle=idle && motion.status(true).signedPwm==0; }
    check(idle,"Encoderrauschen nach Feinhalt erzeugt keine erneuten Fahrimpulse");
  }
}

static void testDownwardNoiseTorqueBound()
{
  MotionController motion; initialize(motion);
  check(motion.command(0,-2.0,500,500), "Abwaerts-Anlauf mit kleinen unbestaetigten Encoderbewegungen angenommen");
  const double noise[] = {-0.11,-0.088,0.0,0.088,0.11,0.044,0.0,-0.044};
  bool bounded=true;
  for(uint32_t now=200;now<3200 && !motion.fault();now+=20)
  {
    motion.tick(now,0,noise[(now/20)%8],true,true);
    int pwm=motion.status(true).signedPwm;
    bounded=bounded && pwm<=0 && pwm>=-34;
  }
  check(bounded, "Unbestaetigte Abwaertsbewegung kann die Anlaufbegrenzung nicht durch Lastkorrektur umgehen");
  check(motion.fault() && motion.status(true).signedPwm==0,
    "Encoderstoerung ohne realen Fortschritt behaelt die unabhaengige Abschaltung");
}

static void testConfigurableLoadedDownwardStart()
{
  // Hardware 2026-10-12: a stationary loaded axis did not break away at
  // -34 for the complete 1.5 s start budget. Model the threshold separately
  // from running friction; an adjustable start limit must not raise cruise
  // bias, shorten quiet confirmation or weaken the independent watchdog.
  for(int startPwm : {34,38})
  {
    MotionController motion; initialize(motion,0,8.55);
    MotionSettings settings=motion.settings();
    check(settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,startPwm*1000),"Anlaufparameter akzeptiert ausreichende Losbrechkraft");
    check(motion.configure(settings),"Abwaerts-Anlaufgrenze bis zur vorhandenen Gesamtgrenze konfigurierbar");
    check(motion.command(0,6.55,200,200),"Beladener Zwei-Grad-Abwaertsschritt angenommen");
    FrictionTiltDrive drive(8.55,1.6,22.0,19.0);
    bool bounded=true;
    for(uint32_t now=200;now<12000 && !motion.fault() && !motion.reached();now+=20)
    {
      motion.tick(now,0,drive.position,true,true);
      int pwm=motion.status(true).signedPwm;
      bounded=bounded && pwm<=0 && pwm>=-40;
      if(std::abs(drive.velocity)<0.05 && -pwm<38) drive.velocity=0;
      else drive.step(pwm);
    }
    check(bounded,"Beladener Abwaertsanlauf behaelt Richtung und Gesamtleistungsgrenze");
    check(startPwm==34 ? motion.fault() && std::strcmp(motion.fault(),"MOTION_STALLED")==0 && drive.position==8.55 :
      !motion.fault() && motion.reached() && std::abs(drive.position-6.55)<=0.3,
      "Zu schwacher Anlauf stoppt; ausreichende konfigurierte Losbrechkraft erreicht das ruhige Ziel");
    check(motion.status(true).signedPwm==0,"Erfolg und blockierter Anlauf enden mit abgeschaltetem Motor");
  }
  MotionSettings limits;
  check(!limits.set(MP_TRAVEL_DOWN_START_PWM_MAX,50001),"Abwaerts-Anlauf oberhalb PWM 50 bleibt ungueltig");
  limits.set(MP_TRAVEL_DOWN_START_PWM_MAX,38000); limits.set(MP_TRAVEL_PWM_MAX,37000);
  check(limits.validate()!=0,"Abwaerts-Anlauf darf die vom Benutzer reduzierte Gesamtgrenze nicht umgehen");
}

static void testLoadedDownwardSpeed()
{
  // Loaded trace: PWM -21 sustains roughly 5 dps after breakaway. The
  // existing floor must not prevent regulation to the requested 2.5 dps.
  // Include the later trace, still about 3.2 dps at the reduced PWM -17.
  for(int load=0;load<2;++load)
  {
  MotionController motion; initialize(motion);
  FrictionTiltDrive drive(0,load==0 ? 2.5 : 1.6,22.0,load==0 ? 19.0 : 15.0);
  check(motion.command(0,-10.0,500,500), "Schnelle laufende Kamera-Abwaertsreaktion angenommen");
  double sum=0; unsigned int samples=0; bool boundedForward=true;
  for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
  {
    motion.tick(now,0,drive.position,true,true);
    AxisMotionStatus status=motion.status(true);
    boundedForward=boundedForward && status.signedPwm<=0 && status.signedPwm>=-40;
    drive.step(status.signedPwm);
    if(status.driveStage==DRIVE_CRUISE && now>=1700 && drive.position>-9.5)
    { sum-=drive.velocity; ++samples; }
  }
  double speed=samples ? sum/samples : 0;
  if(speed<2.0 || speed>3.0 || motion.fault())
    std::cerr << "CameraDownSpeed mean=" << speed << " samples=" << samples
      << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
  check(samples>=10 && speed>=2.0 && speed<=3.0,
    "Kameralast regelt Abwaertstempo trotz schneller Reaktion unterhalb der alten PWM-Untergrenze");
  check(boundedForward && motion.reached() && !motion.fault() &&
    std::abs(drive.position+10.0)<=0.3 && motion.status(true).signedPwm==0,
    "Langsameres Abwaertstempo behaelt Richtung, Zielruhe und PWM-Limit");
  }
}

static void testLoadedUpwardCruiseConsistency()
{
  // Running torque above the unloaded bias, with the observed encoder
  // quantization and small alternating disturbances. Judge actual motion.
  for(double breakaway : {28.0,35.0})
  for(unsigned int noisePhase=0;noisePhase<2;++noisePhase)
  {
  MotionController motion; initialize(motion);
  FrictionTiltDrive drive(0,1.3,22.0,25.0,breakaway);
  check(motion.command(0,10.0,500,500), "Kamera-Aufwaertsfahrt fuer laufende Gleichmaessigkeit angenommen");
  double minimum=100, maximum=0; unsigned int samples=0;
  unsigned int startupPause=0, maximumStartupPause=0;
  const double noise[] = {-0.11,0.11,-0.11,0.11};
  for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
  {
    double measured=std::round(drive.position/0.022)*0.022+
      (motion.status(true).signedPwm!=0 ? noise[(now/20+noisePhase)%4] : 0);
    motion.tick(now,0,measured,true,true);
    AxisMotionStatus status=motion.status(true);
    drive.step(status.signedPwm);
    if(drive.position>0.1 && drive.position<2.0)
    {
      startupPause=drive.velocity<1.0 ? startupPause+20 : 0;
      if(startupPause>maximumStartupPause) maximumStartupPause=startupPause;
    }
    if(status.driveStage==DRIVE_CRUISE && drive.position>2.0 && drive.position<8.0)
    { minimum=fmin(minimum,drive.velocity); maximum=fmax(maximum,drive.velocity); ++samples; }
  }
  if(minimum<1.0 || maximum>4.0 || motion.fault())
    std::cerr << "CameraUpCruise min=" << minimum << " max=" << maximum
      << " samples=" << samples << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
  check(samples>=10 && minimum>=1.0 && maximum<=4.0,
    "Laufende Kamera-Aufwaertsfahrt wechselt nicht zwischen nahezu Stillstand und starker Beschleunigung");
  if(maximumStartupPause>=160) std::cerr << "CameraUpHandoff pauseMs=" << maximumStartupPause << std::endl;
  check(maximumStartupPause<160,
    "Auch der Lastuebergang nach erstem Anlaufen faellt nicht laenger unter 40 Prozent des Solltempos");
  check(motion.reached() && !motion.fault() && std::abs(drive.position-10.0)<=0.3 && motion.status(true).signedPwm==0,
    "Gleichmaessigere Lastfahrt behaelt das unveraenderte Zielfenster");
  }
}

static void testLoadedUpwardNoisySpeed()
{
  // Adjacent encoder disturbances need not alternate at every sample.
  // A short weak slope during already fast cruise must not add torque.
  MotionController motion; initialize(motion);
  FrictionTiltDrive drive(0,1.3,22.0,25.0);
  check(motion.command(0,10.0,500,500), "Kamera-Aufwaertsfahrt mit korrelierten Encoderstoerungen angenommen");
  const double noise[] = {-0.11,-0.11,0.11,0.11};
  double sum=0; unsigned int samples=0; bool boundedForward=true;
  for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
  {
    double measured=std::round(drive.position/0.022)*0.022+
      (motion.status(true).signedPwm!=0 ? noise[(now/20)%4] : 0);
    motion.tick(now,0,measured,true,true);
    AxisMotionStatus status=motion.status(true);
    boundedForward=boundedForward && status.signedPwm>=0 && status.signedPwm<=40;
    // Retain the passive loaded coast used by the other camera fixtures.
    if(status.signedPwm==0 && drive.velocity>0)
    { drive.velocity*=std::exp(-0.02/0.10); drive.position+=drive.velocity*0.02; }
    else drive.step(status.signedPwm);
    if(status.driveStage==DRIVE_CRUISE && drive.position>2.0 && drive.position<8.0)
    { sum+=drive.velocity; ++samples; }
  }
  double mean=samples ? sum/samples : 0;
  if(mean<2.0 || mean>3.0 || motion.fault())
    std::cerr << "CameraUpNoisySpeed mean=" << mean << " samples=" << samples
      << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
  check(samples>=10 && mean>=2.0 && mean<=3.0,
    "Kurze verrauschte Aufwaerts-Rueckmeldung erhoeht kein bereits zu schnelles laufendes Tempo");
  if(!motion.reached() || motion.fault() || std::abs(drive.position-10.0)>0.3)
    std::cerr << "CameraUpNoisyArrival position=" << drive.position
      << " velocity=" << drive.velocity << " pwm=" << motion.status(true).signedPwm
      << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
  check(boundedForward && motion.reached() && !motion.fault() &&
    std::abs(drive.position-10.0)<=0.3 && motion.status(true).signedPwm==0,
    "Verrauschte Last-Temporegelung behaelt Richtung, Zielfenster und Abschaltung");
}

static void testLoadedUpwardRelease()
{
  // Real load first sticks, then accelerates while accumulated torque remains.
  for(double distance : {2.0,10.0})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,0.8,29.0,25.0);
    check(motion.command(0,distance,500,500), "Kamera-Aufwaertsfahrt mit abnehmender laufender Reibung angenommen");
    double peak=0; bool boundedForward=true;
    const double noise[] = {-0.044,0.022,0.044,-0.022};
    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      if(drive.position>0.2) drive.positiveFriction=13.0;
      double measured=std::round(drive.position/0.022)*0.022+noise[(now/20)%4];
      motion.tick(now,0,measured,true,true);
      int pwm=motion.status(true).signedPwm;
      boundedForward=boundedForward && pwm>=0 && pwm<=40;
      if(pwm==0 && drive.velocity>0)
      { drive.velocity*=std::exp(-0.02/0.10); drive.position+=drive.velocity*0.02; }
      else drive.step(pwm);
      peak=fmax(peak,std::abs(drive.velocity));
    }
    if(motion.fault() || !motion.reached())
      std::cerr << "CameraRelease distance=" << distance << " peak=" << peak
                << " position=" << drive.position << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(boundedForward && motion.reached() && !motion.fault(), "Lastwechsel behaelt Zielruhe, Richtung und bestehendes PWM-Limit");
    check(std::abs(drive.position-distance)<=0.3 && motion.status(true).signedPwm==0,
      "Aufwaerts-Auslauf behaelt unveraendertes Zielfenster und Leistungsabschaltung");
  }
}
static void testLoadedUpwardTorqueReserve()
{
  // Loaded trace remains stationary at PWM 28 despite available cap 40.
  for(double distance : {2.0,10.0})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,0.8,29.0,25.0);
    check(motion.command(0,distance,500,500), "Aufwaertsfahrt mit Kameralast oberhalb PWM 28 angenommen");
    bool boundedForward=true;
    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      motion.tick(now,0,drive.position,true,true);
      int pwm=motion.status(true).signedPwm;
      boundedForward=boundedForward && pwm>=0 && pwm<=40;
      drive.step(pwm);
    }
    if(motion.fault() || !motion.reached())
      std::cerr << "CameraReserve distance=" << distance << " position=" << drive.position
                << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(motion.reached() && !motion.fault(), "Hoehere Kameralast nutzt vorhandene Fahrleistung statt an PWM 28 zu verharren");
    check(boundedForward && std::abs(drive.position-distance)<=0.3 && motion.status(true).signedPwm==0,
      "Hoehere laufende Reserve behaelt PWM-Limit, Richtung und stromlose Zielruhe");
  }
}
static void testCameraRecoveryOverspeed()
{
  MotionController motion; initialize(motion);
  FrictionTiltDrive drive(0,1.2,22.0,19.0);
  check(motion.command(0,-2.0,500,500), "Kamera-Abwaertsfahrt mit verrauschtem Anlauf angenommen");
  double peak=0;
  const double noise[] = {-0.044,0.022,0.044,-0.022};
  for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
  {
    double measured=std::round(drive.position/0.022)*0.022+noise[(now/20)%4];
    motion.tick(now,0,measured,true,true);
    int pwm=motion.status(true).signedPwm;
    if(pwm==0 && drive.velocity<0)
    { drive.velocity*=std::exp(-0.02/0.10); drive.position+=drive.velocity*0.02; }
    else drive.step(pwm);
    peak=fmax(peak,std::abs(drive.velocity));
  }
  if(peak>5.0 || motion.fault() || !motion.reached())
    std::cerr << "CameraRecovery peak=" << peak << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
  check(peak<=5.0 && motion.reached() && !motion.fault() && std::abs(drive.position+2.0)<=0.3 && motion.status(true).signedPwm==0,
    "Verrauschter Kamera-Anlauf und Auslauf behalten das Zielfenster ohne Gegenkorrektur");
}
static void testLoadedDownwardCoast()
{
  // Camera trace retains about 0.4 degrees of downhill coast after PWM zero.
  // Model that passive decay separately from powered running friction.
  for(double distance : {1.0,2.0,10.0})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,1.8,22.0,19.0);
    check(motion.command(0,-distance,500,500), "Kamera-Abwaertsfahrt mit passivem Auslauf angenommen");
    bool forwardOnly=true;
    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      motion.tick(now,0,drive.position,true,true);
      int pwm=motion.status(true).signedPwm;
      forwardOnly=forwardOnly && pwm<=0;
      if(pwm==0 && drive.velocity<0)
      { drive.velocity*=std::exp(-0.02/0.10); drive.position+=drive.velocity*0.02; }
      else drive.step(pwm);
    }
    if(motion.fault() || !motion.reached())
      std::cerr << "CameraCoast distance=" << distance << " position=" << drive.position
                << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(forwardOnly && motion.reached() && !motion.fault() &&
      std::abs(drive.position+distance)<=0.3 && motion.status(true).signedPwm==0,
      "Frueheres Abschalten beruecksichtigt Kamera-Auslauf ohne Gegenfahrt oder groesseres Zielfenster");
  }
}
static void testCameraRunningLoad()
{
  // The camera trace breaks away, then stops after dropping to unloaded bias.
  // Exercise running torque above that bias, still inside the existing cap;
  // this bounded regression plant does not identify the camera mechanics.
  for(int direction : {1,-1})
  for(double distance : {2.0,10.0})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,0.8,22.0,25.0);
    check(motion.command(0,direction*distance,500,500), "Tiltfahrt mit hoeherer Kameralast angenommen");
    int pauseMs=0, maximumPauseMs=0; bool boundedForward=true;
    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      motion.tick(now,0,drive.position,true,true);
      AxisMotionStatus status=motion.status(true);
      drive.step(status.signedPwm);
      boundedForward=boundedForward && status.signedPwm*direction>=0 && std::abs(status.signedPwm)<=40;
      if(drive.position*direction>0.05 && drive.position*direction<distance-0.3)
      {
        pauseMs=std::abs(drive.velocity)<0.12 ? pauseMs+20 : 0;
        maximumPauseMs=maximumPauseMs>pauseMs ? maximumPauseMs : pauseMs;
      }
    }
    if(maximumPauseMs>=160 || motion.fault() || !motion.reached())
      std::cerr << "CameraLoad dir=" << direction << " distance=" << distance << " pause=" << maximumPauseMs
                << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(maximumPauseMs<160 && motion.reached() && !motion.fault(),
      "Kameralast bleibt nach dem Losbrechen ohne lange Zwischenpause in Fahrt");
    check(boundedForward && std::abs(drive.position-direction*distance)<=0.3 && motion.status(true).signedPwm==0,
      "Lastanpassung behaelt Leistungslimit, Richtung und stromlose Zielruhe");
  }
}
static void testStartupWithIncreasingRunningFriction()
{
  for(int direction : {1,-1})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,direction>0 ? 0.8 : 1.2,8.0,19.0);
    check(motion.command(0,direction*10.0,500,500), "Anlauf mit steigender laufender Reibung angenommen");
    int pauseMs=0, maximumPauseMs=0;
    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      if(drive.position*direction>1.2) { drive.positiveFriction=13.0; drive.negativeFriction=21.0; }
      motion.tick(now,0,drive.position,true,true);
      drive.step(motion.status(true).signedPwm);
      if(drive.position*direction>0.3 && drive.position*direction<9.7)
      {
        pauseMs=std::abs(drive.velocity)<0.12 ? pauseMs+20 : 0;
        maximumPauseMs=maximumPauseMs>pauseMs ? maximumPauseMs : pauseMs;
      }
    }
    if(maximumPauseMs>=160 || motion.fault() || !motion.reached())
      std::cerr << "FrictionStep dir=" << direction << " pause=" << maximumPauseMs
                << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(maximumPauseMs<160 && motion.reached() && !motion.fault(),
      "Anlauf-Leistungsruecknahme verursacht bei hoeherer laufender Reibung keine Zwischenpause");
    check(std::abs(drive.position-direction*10.0)<=0.3 && motion.status(true).signedPwm==0,
      "Reibungswechsel behaelt das Zielfenster und die finale Leistungsabschaltung");
  }
}
static void testEarlyReliefCannotExtendBlockedStart()
{
  for(int direction : {1,-1})
  {
    MotionController motion; initialize(motion);
    check(motion.command(0,direction*10.0,500,500), "Blockierter Anlauf mit fruehen Encoderhinweisen angenommen");
    double bias=direction>0 ? 15.0 : 25.0;
    bool relief=false, resumed=false, unconfirmed=true;
    uint32_t now=180;
    for(int sample=0;sample<100 && !motion.fault();++sample)
    {
      now+=20;
      int previous=std::abs(motion.status(true).signedPwm);
      double hint=sample%10==0 && previous>=bias ? direction*0.044 : 0;
      motion.tick(now,0,hint,true,true);
      AxisMotionStatus status=motion.status(true);
      if(hint!=0 && std::abs(status.signedPwm)<=bias-MOTION_TRAVEL_PWM_REDUCTION_MAX) relief=true;
      if(relief && std::abs(status.signedPwm)>bias+5) resumed=true;
      unconfirmed=unconfirmed && status.driveStage!=DRIVE_CRUISE && !motion.reached();
    }
    check(relief && resumed && unconfirmed,
      "Frueher Encoderhinweis nimmt kurz Leistung zurueck, bestaetigt aber keinen blockierten Anlauf");
    check(motion.fault() && std::strcmp(motion.fault(),"MOTION_STALLED")==0 &&
      motion.status(true).signedPwm==0 && now-180<=MOTION_TRAVEL_START_TIMEOUT_MS+40,
      "Wiederholte fruehe Hinweise verlaengern weder Anlaufbudget noch Motorfreigabe");
  }
}
static void testStartupVelocityPeak()
{
  for(int direction : {1,-1})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,direction>0 ? 0.8 : 1.2,8.0,19.0);
    check(motion.command(0,direction*10.0,500,500), "Tilt-Anlaufspitzentest angenommen");
    double peak=0; bool forwardOnly=true;

    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      motion.tick(now,0,drive.position,true,true);
      AxisMotionStatus status=motion.status(true);
      drive.step(status.signedPwm);
      if(now<1200) peak=fmax(peak,std::abs(drive.velocity));
      forwardOnly=forwardOnly && status.signedPwm*direction>=0;
    }
    if(peak>3.5 || motion.fault())
      std::cerr << "StartupPeak dir=" << direction << " peak=" << peak
                << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(peak>0.5 && peak<=3.5,
      "Begrenzter Anlauf reduziert die kurzzeitige Geschwindigkeitsspitze im lokalen Messmodell");
    check(forwardOnly && motion.reached() && !motion.fault() &&
      std::abs(drive.position-direction*10.0)<=0.3 && motion.status(true).signedPwm==0,
      "Sanfterer Anlauf erhaelt Zielruhe ohne Gegenfahrt und endgueltige Abschaltung");
  }
}
static void testMeasuredCruiseSpeed()
{
  // Local unloaded measurements around +8..+14 degrees: +11 and -21 keep
  // moving at roughly 2.3..2.6 dps. This bounded regression plant captures
  // the lower running torque, not a complete identification of the rig.
  for(int direction : {1,-1})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,direction>0 ? 0.8 : 1.2,8.0,19.0);
    check(motion.command(0,direction*10.0,500,500), "Gemessene langsame Tiltfahrt angenommen");
    double speedSum=0; unsigned int samples=0; bool continuous=true, forwardOnly=true;
    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      motion.tick(now,0,drive.position,true,true);
      AxisMotionStatus status=motion.status(true);
      drive.step(status.signedPwm);
      if(status.driveStage==DRIVE_CRUISE && now>=1700 && drive.position*direction<9.5)
      { speedSum+=drive.velocity*direction; ++samples; continuous=continuous && drive.velocity*direction>0.25; }
      forwardOnly=forwardOnly && status.signedPwm*direction>=0;
    }
    double speed=samples ? speedSum/samples : 0;
    if(samples<10 || speed<2.15 || speed>2.85 || motion.fault())
      std::cerr << "MeasuredCruise dir=" << direction << " speed=" << speed << " samples=" << samples
                << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(samples>=10 && speed>=2.15 && speed<=2.85,
      "500-Permille-Fahrt regelt nach Anlauf auf etwa 2.5 Grad pro Sekunde statt am PWM-Boden zu bleiben");
    check(continuous && forwardOnly && motion.reached() && !motion.fault() &&
      std::abs(drive.position-direction*10.0)<=0.3 && motion.status(true).signedPwm==0,
      "Tempoanpassung erhaelt kontinuierliche Fahrt und stromlosen Halt ohne Gegenfahrt");
  }
}
static void testFullFrictionTravel()
{
  // Range measurements show much faster motion once the gear has broken away
  // than the original regression plant. Include the first and last degree,
  // where the real 2.4.13 trace stopped and subsequently broke away again.
  for(int direction : {1,-1})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0,direction>0 ? 1.3 : 1.0);
    check(motion.command(0,direction*10.0,500,500), "Gesamte Reibungsfahrt angenommen");
    int pauseMs=0, maximumPauseMs=0; bool began=false;
    double backwards=0, last=drive.position;
    for(uint32_t now=200;now<20200 && !motion.fault() && !motion.reached();now+=20)
    {
      double noise=std::abs(motion.status(true).signedPwm)>0 ? ((now/20)%2 ? 0.11 : -0.11) : 0;
      motion.tick(now,0,std::round(drive.position/0.022)*0.022+noise,true,true);
      drive.step(motion.status(true).signedPwm);
      double progress=drive.position*direction;
      if(progress>0.3 && progress<9.7)
      {
        if(std::abs(drive.velocity)>0.25) began=true;
        pauseMs=began && std::abs(drive.velocity)<0.12 ? pauseMs+20 : 0;
        if(pauseMs>maximumPauseMs) maximumPauseMs=pauseMs;
      }
      if((drive.position-last)*direction<0) backwards+=std::abs(drive.position-last);
      last=drive.position;
    }
    if(maximumPauseMs>=160 || motion.fault() || !motion.reached())
      std::cerr << "FullTravel dir=" << direction << " pauseMs=" << maximumPauseMs
        << " actual=" << drive.position << " backwards=" << backwards
        << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(maximumPauseMs<160, "Auch erste und letzte Fahrstrecke haben keine erneute Losbrechpause");
    check(motion.reached() && !motion.fault() && std::abs(drive.position-direction*10)<=0.3,
      "Gesamte Fahrt erreicht das experimentelle 0.3-Grad-Fenster mit bestaetigter Ruhe");
    check(backwards<0.1, "Ankunft erzeugt keine sichtbare Gegenfahrt");
  }
}

static void testTravelRangeAndStop()
{
  for(double gain : {0.8,1.0,1.3})
  for(double distance : {0.5,1.0,10.0,55.0})
  for(int direction : {1,-1})
  {
    double start=direction>0 ? -30.0 : 25.0;
    MotionController motion; initialize(motion,0,start);
    FrictionTiltDrive drive(start,gain);
    double target=start+direction*distance;
    check(motion.command(0,target,500,500), "Kleine und grosse Bereichsfahrt angenommen");
    int pauseMs=0, maximumPause=0; bool moving=false, bounds=true;
    uint32_t now=180;
    for(int sample=0;sample<4000 && !motion.reached() && !motion.fault();++sample)
    {
      now+=20; motion.tick(now,0,drive.position,true,true);
      int pwm=motion.status(true).signedPwm;
      drive.step(pwm);
      bounds=bounds && drive.position>=-45 && drive.position<=30 && std::abs(pwm)<=40;
      double progress=(drive.position-start)*direction;
      if(progress>0.3 && progress<distance-0.3)
      {
        if(drive.velocity*direction>0.25) moving=true;
        pauseMs=moving && std::abs(drive.velocity)<0.12 ? pauseMs+20 : 0;
        if(pauseMs>maximumPause) maximumPause=pauseMs;
      }
    }
    if(!motion.reached() || motion.fault()) std::cerr << "Range gain=" << gain << " distance=" << distance
      << " dir=" << direction << " actual=" << drive.position << " target=" << target
      << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(bounds && !motion.fault() && motion.reached() && std::abs(drive.position-target)<=0.3,
      "0.5 bis 55 Grad finden das Zielfenster in beiden Richtungen ohne Grenzverletzung");
    check(maximumPause<160 && motion.status(true).signedPwm==0,
      "Bereichsfahrt bleibt kontinuierlich bis zum finalen stromlosen Halt");
    motion.disable(); now+=20; motion.tick(now,0,drive.position,true,true);
    check(motion.status(true).signedPwm==0 && motion.status(true).driveStage==DRIVE_IDLE,
      "Stop loescht den Phasenauftrag auch nach erreichter Position");
  }
  for(int stopAt : {2,12,40})
  {
    MotionController motion; initialize(motion); motion.command(0,10,500,500);
    FrictionTiltDrive drive(0); uint32_t now=180;
    for(int sample=0;sample<stopAt;++sample)
    { now+=20; motion.tick(now,0,drive.position,true,true); drive.step(motion.status(true).signedPwm); }
    motion.disable(); now+=20; motion.tick(now,0,drive.position,true,true);
    check(!motion.fault() && motion.status(true).signedPwm==0 && motion.status(true).driveStage==DRIVE_IDLE,
      "Stop im Anlauf oder in der Fahrt nimmt Leistung sofort weg und startet nicht neu");
  }
}

static void testContinuousFrictionTilt()
{
  for(int direction : {1,-1})
  {
    MotionController motion; initialize(motion);
    FrictionTiltDrive drive(0);
    check(motion.command(0,direction*10.0,500,500), "Reibungsmodell: Zehn-Grad-Fahrt angenommen");
    int interrupted=0; bool moving=false;
    for(uint32_t now=200;now<30200 && !motion.fault() && !motion.reached();now+=20)
    {
      // The real idle traces contain increments up to roughly +/-0.11 deg.
      double noise=motion.status(true).signedPwm==0 ? 0 : ((now/20)%2 ? 0.11 : -0.11);
      double encoder=std::round(drive.position/0.044)*0.044+noise;
      motion.tick(now,0,encoder,true,true);
      drive.step(motion.status(true).signedPwm);
      double progress=drive.position*direction;
      if(progress>2 && progress<8)
      {
        bool running=drive.velocity*direction>0.1;
        if(moving && !running) ++interrupted;
        moving=running;
      }
    }
    if(interrupted || motion.fault() || std::abs(drive.position-direction*10.0)>MOTION_TRAVEL_POSITION_ENTER_DEG)
      std::cerr << "FrictionTilt direction=" << direction << " interruptions=" << interrupted
                << " position=" << drive.position << " fault=" << (motion.fault()?motion.fault():"none") << std::endl;
    check(interrupted==0, "Laufende Tiltfahrt faellt zwischen 2 und 8 Grad nicht wieder in Haftreibung");
    // Moving encoder noise is removed after power-off; actual rest still has
    // to meet the independent unchanged span/velocity confirmation checks.
    check(std::abs(drive.position-direction*10.0)<=MOTION_TRAVEL_POSITION_ENTER_DEG && !motion.fault(),
      "Reibungsmodell erreicht das Tiltziel trotz Encoderrauschen");
    check(motion.reached() && !motion.fault(), "Ruhige Rueckmeldung bestaetigt das erreichte Tiltziel");
  }
}

static void testTravelFaultsAndReplan()
{
  MotionController stalled; initialize(stalled); stalled.command(0,10,500,500);
  uint32_t now=180;
  // Genuine startup progress, then a blocked output. The running floor must
  // not disable stall detection merely because it is below legacy PWM 30.
  for(int sample=0;sample<20 && !stalled.fault();++sample)
  { now+=20; stalled.tick(now,0,sample*0.03,true,true); }
  check(stalled.status(true).driveStage==DRIVE_CRUISE, "Abtriebsfortschritt wechselt zur Fahrphase");
  for(int sample=0;sample<150 && !stalled.fault();++sample)
  { now+=20; stalled.tick(now,0,0.57,true,true); }
  check(stalled.fault() && std::strcmp(stalled.fault(),"MOTION_STALLED")==0 && stalled.status(true).signedPwm==0,
    "Stillstand waehrend kontinuierlicher Fahrt schaltet ab statt erneut anzulaufen");

  MotionController noisy; initialize(noisy); noisy.command(0,10,500,500);
  now=180;
  for(int sample=0;sample<350 && !noisy.fault();++sample)
  { now+=20; noisy.tick(now,0,10+(sample%2 ? 0.11 : -0.11),true,true); }
  check(noisy.fault() && std::strcmp(noisy.fault(),"MOTION_NOT_QUIET")==0 && noisy.status(true).signedPwm==0,
    "Zu grosse Ruhebewegung bleibt Fehler ohne weitere Fahrimpulse oder Fertigmeldung");

  MotionController changing; initialize(changing); changing.command(0,10,500,500);
  FrictionTiltDrive drive(0); now=180;
  for(int sample=0;sample<40 && !changing.fault();++sample)
  { now+=20; changing.tick(now,0,drive.position,true,true); drive.step(changing.status(true).signedPwm); }
  check(changing.command(0,-5,500,500), "Richtungswechsel waehrend Phasenfahrt angenommen");
  for(int sample=0;sample<1000 && !changing.fault() && !changing.reached();++sample)
  { now+=20; changing.tick(now,0,drive.position,true,true); drive.step(changing.status(true).signedPwm); }
  check(changing.reached() && !changing.fault() && std::abs(drive.position+5)<=0.3,
    "Umplanung wartet Gegenbewegung ab und erreicht das neue Ziel ohne alten Auftrag");
}

static void testNoisyTiltStall()
{
  MotionController motion; initialize(motion);
  check(motion.command(0,0.32,500,500), "Kleine Tiltkorrektur angenommen");
  bool bounded=true;
  for(uint32_t now=200;now<6200 && !motion.fault();now+=20)
  {
    motion.tick(now,0,(now/20)%2 ? 0.11 : -0.11,true,true);
    bounded=bounded && std::abs(motion.status(true).signedPwm)<=MOTION_REVERSAL_PWM_MAX;
  }
  check(bounded && motion.fault() && motion.status(true).signedPwm==0,
    "Ruherauschen taeuscht bei blockierter kleiner Zielkorrektur keinen dauerhaften Fortschritt vor");
}

static void testTrackingPauseDirection()
{
  for (int direction : {-1, 1})
  {
    MotionController motion; initialize(motion);
    motion.command(0, direction * 10.0, 500, 500);
    uint32_t now=180;
    while(std::abs(motion.status(true).reference)<4.0 && now<6000)
    {
      double position=motion.status(true).reference;
      now+=20; motion.tick(now,0,position,true,true);
    }
    double reference=motion.status(true).reference;
    // Recorded failure: output leads by 0.8 degrees and then stops. Freezing
    // the reference while retaining its velocity creates a permanent balance.
    double ahead=reference+direction*0.8;
    for(int i=0;i<10;++i) { now+=20; motion.tick(now,0,ahead,true,true); }
    check(!motion.fault() && (motion.status(true).reference-reference)*direction>0.3,
      "Vorauseilende Achse laesst die Sollkurve in beiden Richtungen weiterlaufen");

    reference=motion.status(true).reference;
    double behind=reference-direction*0.8;
    for(int i=0;i<10;++i) { now+=20; motion.tick(now,0,behind,true,true); }
    check(!motion.fault() && std::abs(motion.status(true).reference-reference)<1e-9,
      "Zurueckbleibende Achse behaelt die Begrenzung des Sollvorlaufs");
  }
}

static void testBoundedBenchDrive()
{
  BenchDrive bench;
  check(!bench.start(true,40,501,2000,0,0,0,true) && !bench.start(true,-40,501,2000,0,0,0,true),
    "Hoehere Losbrech-Messleistung bleibt in beiden Richtungen auf 500 ms begrenzt");
  check(!bench.start(true,41,1000,2000,0,0,0,true) &&
        !bench.start(true,0,1000,2000,0,0,0,true) &&
        !bench.start(true,20,3001,2000,0,0,0,true) &&
        !bench.start(true,20,1000,5001,0,0,0,true) &&
        !bench.start(true,20,1000,2000,0,0,0,false),
    "Messfahrt lehnt ungueltige Leistung, Zeit, Weg und veraltete Sensoren ab");
  check(!bench.start(true,20,1000,2000,0,0,29000,true), "Messfahrt braucht Platz innerhalb gespeicherter Limits");
  check(!bench.start(true,30,1000,3000,0,0,23000,true), "Messweg behaelt fuenf Grad Reserve fuer gemessenen Auslauf");
  check(bench.start(true,30,1000,2000,0,0,0,true), "Begrenzte Tilt-Messfahrt startet");
  check(!bench.start(false,20,1000,2000,0,0,0,true), "Aktive Messfahrt kann nicht ersetzt werden");
  bench.tick(20,0,0,true,true);
  check(bench.drive(true)==5 && bench.drive(false)==0, "Mess-PWM steigt sanft, andere Achse bleibt aus");
  for(uint32_t now=40;now<=1000;now+=20) bench.tick(now,0,100,true,true);
  check(!bench.active() && bench.drive(true)==0 && std::strcmp(bench.status().reason,"TIME_BOUND")==0,
    "Firmware beendet Messleistung am Zeitlimit auch ohne Hoststop");
  for(uint32_t now=1020;now<=1500;now+=20) bench.tick(now,0,100,true,false);
  BenchDriveSample sample;
  check(!bench.status().recording && bench.status().count==76 && bench.sample(75,sample) &&
      sample.elapsedMs==1500 && sample.signedPwm==0 && !bench.sample(76,sample),
    "Regeltaktspur enthaelt 500 ms unbestromten Nachlauf und bleibt indexbegrenzt");
  check(bench.start(false,-30,3000,5000,2000,0,0,true), "Pan-Messfahrt in Gegenrichtung startet");
  bench.tick(2020,-5000,0,true,true);
  check(!bench.active() && bench.drive(false)==0 && std::strcmp(bench.status().reason,"TRAVEL_BOUND")==0,
    "Weggrenze beendet die Messfahrt sofort");
  bench.start(true,20,1000,2000,2100,0,0,true); bench.tick(2120,0,0,false,true);
  check(!bench.active() && bench.drive(true)==0, "Sensorfehler nimmt Messleistung weg");
  bench.start(true,20,1000,2000,2200,0,0,true); bench.tick(2320,0,0,true,true);
  check(!bench.active() && std::strcmp(bench.status().reason,"CONTROL_TIMEOUT")==0,
    "Ausgefallener Regeltakt beendet die Messfahrt");
  bench.start(true,20,1000,2000,2400,0,0,true); bench.tick(2420,0,0,true,false);
  check(!bench.active() && bench.drive(true)==0, "Entzug der Freigabe beendet Messleistung");
  bench.start(true,20,1000,2000,UINT32_MAX-10,0,0,true); bench.tick(9,0,0,true,true);
  check(bench.active() && bench.drive(true)==5, "Messfahrt verkraftet Zeitzaehlerueberlauf");
  bench.stop(10,"TEST_STOP"); bench.tick(30,0,0,true,true);
  check(!bench.active() && bench.drive(true)==0, "Stop nimmt alte Messleistung nicht wieder auf");
}

static void testBenchCruiseStages()
{
  BenchDrive bench;
  check(!bench.startCruise(true,31,2000,5000,0,0,0,true) &&
    !bench.startCruise(true,-31,2000,5000,0,0,0,true) &&
    !bench.startCruise(true,15,599,5000,0,0,0,true) &&
    !bench.startCruise(true,15,2000,999,0,0,0,true),
    "Anlaufmessung begrenzt laufende PWM, Gesamtzeit und Mindestweg");
  for(int direction : {-1,1})
  {
    BenchDrive drive;
    check(drive.startCruise(true,direction*15,2000,5000,0,0,0,true), "Anlaufmessung startet in beiden Richtungen");
    BenchDriveSample sample;
    drive.sample(0,sample);
    check(sample.stage==1 && sample.signedPwm==0, "Anlaufphase ist ab der ersten Probe gekennzeichnet");
    for(uint32_t now=20;now<=160;now+=20) drive.tick(now,0,direction*220,true,true);
    drive.sample(drive.status().count-1,sample);
    check(sample.stage==1 && drive.drive(true)==direction*40, "Ruherauschen wechselt nicht vorzeitig zur kleinen PWM");
    drive.tick(180,0,direction*500,true,true);
    drive.sample(drive.status().count-1,sample);
    check(sample.stage==2 && drive.drive(true)==direction*35 && drive.drive(false)==0,
      "Encoderfortschritt startet sanften PWM-Uebergang ohne Zwischenstop");
    for(uint32_t now=200;now<=400;now+=20) drive.tick(now,0,direction*1000,true,true);
    check(drive.active() && drive.drive(true)==direction*15, "Laufende Messung haelt kleine PWM auch nach nachfolgendem Stillstand");
    drive.tick(420,0,direction*5000,true,true);
    drive.sample(drive.status().count-1,sample);
    check(!drive.active() && sample.stage==3 && sample.signedPwm==0 && std::strcmp(drive.status().reason,"TRAVEL_BOUND")==0,
      "Gemeinsame Weggrenze beendet Anlauf und Weiterfahrt mit unbestromter Spur");
  }
  BenchDrive stalled;
  stalled.startCruise(true,-20,2000,5000,0,0,0,true);
  for(uint32_t now=20;now<=500;now+=20) stalled.tick(now,0,0,true,true);
  check(!stalled.active() && stalled.drive(true)==0 && std::strcmp(stalled.status().reason,"START_TIMEOUT")==0,
    "Fehlender Anlauf bricht nach 500 ms ohne Leistungserhoehung ab");
  BenchDrive late;
  late.startCruise(true,10,2000,5000,0,0,0,true);
  for(uint32_t now=20;now<=460;now+=20) late.tick(now,0,0,true,true);
  late.tick(480,0,500,true,true); late.tick(500,0,600,true,true);
  check(late.active() && late.drive(true)<=30, "Auch ein spaeter Start verlaengert hohe PWM nicht ueber 500 ms");
  BenchDrive wrong;
  wrong.startCruise(true,20,2000,5000,0,0,0,true); wrong.tick(20,0,-500,true,true);
  check(!wrong.active() && std::strcmp(wrong.status().reason,"WRONG_DIRECTION")==0, "Entgegengesetzter Encoderfortschritt beendet Anlaufmessung");
  BenchDrive interrupted;
  interrupted.startCruise(true,20,2000,5000,0,0,0,true);
  interrupted.tick(20,0,500,true,true); interrupted.stop(21,"TEST_STOP"); interrupted.tick(40,0,700,true,true);
  check(!interrupted.active() && interrupted.drive(true)==0, "Stop waehrend Weiterfahrt nimmt Anlaufmessung nicht wieder auf");
}

static void testManualPanLoadedStart()
{
  // Same loaded drive for the working arrow and finite Plan target.
  // No encoder progress below PWM 45, then running friction PWM 25.
  for(int direction : {-1,1}) {
    for(bool target : {false,true}) {
      MotionController motion; initialize(motion);
      MotionSettings config=motion.settings(); config.set(MP_PAN_JOG_START_PWM,50000); motion.configure(config);
      check(target ? motion.commandManual(direction*10.0,0,200,200) : motion.commandJogVelocity(false,direction,1.6),
        "Pan-Anlaufvergleich nutzt echte Plan- und Pfeil-Fahrbefehle");
      double position=0,velocity=0; unsigned startMs=0;
      for(uint32_t now=200;now<2200 && !motion.fault();now+=20) {
        motion.tick(now,position,0,true,true);
        int pwm=motion.status(false).signedPwm;
        if(std::abs(velocity)<0.05 && std::abs(pwm)<45) velocity=0;
        else {
          double desired=direction*std::max(0.0,(direction*pwm-25)*0.35);
          velocity+=(desired-velocity)*0.02/0.18;
          position+=velocity*0.02;
        }
        if(!startMs && position*direction>=0.05) startMs=now-180;
      }
      if(!startMs || motion.fault()) std::cerr<<"Pan loaded target="<<target<<" dir="<<direction<<" start="<<startMs<<" pos="<<position<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
      check(startMs>0 && startMs<=1200 && !motion.fault(),
        "Pan-Planfahrt ueberwindet dieselbe Haftreibung wie die Pfeiltaste ohne Stillstandsfehler");
    }
  }
}

static void testManualPanLoadedArrival()
{
  for(int direction : {-1,1}) for(double distance : {1.0,3.0,10.0}) for(bool noisy : {false,true}) for(double runningFriction : {25.0,40.0}) {
    MotionController motion; initialize(motion);
    MotionSettings config=motion.settings(); config.set(MP_PAN_JOG_START_PWM,50000); motion.configure(config);
    motion.commandManual(direction*distance,0,200,200);
    double position=0,velocity=0; bool moving=false; unsigned pause=0,longest=0; int previous=0;
    bool slew=true; double maximumReference=0;
    for(uint32_t now=200;now<22000 && !motion.fault() && !motion.reached();now+=20) {
      double noise[]={-0.022,0.044,-0.044,0.022,0};
      motion.tick(now,position+(noisy ? noise[(now/20)%5] : 0),0,true,true);
      AxisMotionStatus state=motion.status(false); int pwm=state.signedPwm;
      if(std::abs(pwm)>std::abs(previous)) slew=slew && std::abs(pwm)-std::abs(previous)<=1;
      previous=pwm; maximumReference=std::max(maximumReference,std::abs(state.referenceVelocity));
      if(std::abs(velocity)<0.05 && std::abs(pwm)<45) velocity=0;
      else {
        int d=velocity==0 ? direction : (velocity>0 ? 1 : -1);
        double next=velocity+(pwm-d*runningFriction-velocity/0.35)*0.02/0.18;
        velocity=next*d<0 ? 0 : next; position+=velocity*0.02;
      }
      if(velocity*direction>0.5) moving=true;
      if(moving && (distance-direction*position)>0.5 && std::abs(velocity)<0.05) { pause+=20; longest=std::max(longest,pause); } else pause=0;
    }
    if(!motion.reached() || motion.fault() || longest>100)
      std::cerr<<"Pan arrival dir="<<direction<<" distance="<<distance<<" noisy="<<noisy<<" pos="<<position<<" pause="<<longest<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(motion.reached() && !motion.fault() && std::abs(direction*distance-position)<=0.3,
      "Beladene Pan-Planfahrt erreicht kurze und lange Ziele in beiden Richtungen auch mit Encoderrauschen");
    check(slew && maximumReference<=1.60001 && longest<=100 && motion.status(false).signedPwm==0,
      "Pan-Zielfahrt steigt sanft an, respektiert Fahrtempo und stockt vor dem Ziel nicht");
  }
}

static void testManualPanSteadyVelocity()
{
  for(int direction : {-1,1}) for(bool noisy : {false,true}) {
    MotionController motion; initialize(motion);
    MotionSettings config=motion.settings(); config.set(MP_PAN_JOG_START_PWM,50000); config.set(MP_PAN_JOG_P_GAIN,500); motion.configure(config);
    motion.commandManual(direction*30.0,0,200,200);
    double position=0,velocity=0,sum=0,squares=0,minimum=100,maximum=0; unsigned count=0;
    for(uint32_t now=200;now<13000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,position+(noisy ? 0.10*std::sin(now*0.012) : 0),0,true,true);
      double pwm=motion.status(false).signedPwm*direction;
      if(velocity<0.05 && pwm<45) velocity=0;
      else { double next=velocity+(pwm-34-velocity/1.3)*0.02/0.18; velocity=std::max(0.0,next); }
      position+=direction*velocity*0.02;
      if(now>=4000 && (30-position*direction)>3) {
        sum+=velocity; squares+=velocity*velocity; minimum=std::min(minimum,velocity); maximum=std::max(maximum,velocity); ++count;
      }
    }
    double mean=count ? sum/count : 0;
    double spread=count ? std::sqrt(std::max(0.0,squares/count-mean*mean))/std::max(0.001,mean) : 100;
    std::cerr<<"Pan steady dir="<<direction<<" noise="<<noisy<<" mean="<<mean<<" cv="<<spread<<" min="<<minimum<<" max="<<maximum<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(!motion.fault() && count>=100 && minimum>=0.6 && mean>=1.2 && mean<=2.0 && spread<=0.25,
      "Pan-Zielfahrt bleibt nach dem Anlauf bei gleichmaessiger Geschwindigkeit ohne Leistungs-/Geschwindigkeitspulse");
  }
}

int runLoadedTiltReserveRegressions()
{
  int before=motionChecks;
  for(int direction : {1,-1}) for(int maximum : {40,50}) {
    MotionController motion; initialize(motion);
    MotionSettings settings=motion.settings();
    bool configured=settings.set(MP_TRAVEL_PWM_MAX,maximum*1000) &&
      settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,maximum*1000) &&
      settings.set(MP_TRAVEL_DOWN_I_MAX,(maximum-25)*1000) && motion.configure(settings);
    check(configured,"Tilt-Leistungsreserve muss explizit konfigurierbar sein");
    check(motion.commandPositioning(0,direction*10.0,500,500),"Beladene Tiltfahrt nutzt den echten Aufnahme-Fahrbefehl");
    double position=0,velocity=0; int peak=0;
    for(uint32_t now=200;now<30000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,0,direction*position,true,true);
      int pwm=motion.status(true).signedPwm*direction; peak=std::max(peak,pwm);
      if(velocity<0.05 && pwm<45) velocity=0;
      else velocity=std::max(0.0,velocity+(pwm-36-velocity/0.8)*0.02/0.18);
      position+=velocity*0.02;
    }
    std::cout<<"Loaded tilt dir="<<direction<<" cap="<<maximum<<" position="<<position<<" peakPwm="<<peak
      <<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    if(maximum==40) check(motion.fault() && std::strcmp(motion.fault(),"MOTION_STALLED")==0 && position==0,
      "Zu schwache Tilt-Leistungsgrenze reproduziert den Anlaufstillstand statt den Schutz zu verlaengern");
    else check(!motion.fault() && motion.reached() && std::abs(position-10)<=0.3 && peak<=50 && motion.status(true).signedPwm==0,
      "Konfigurierte Tilt-Reserve ueberwindet die Last und behaelt Zielruhe und Leistungsgrenze");
  }
  for(int direction : {-1,1}) {
    MotionController blocked; initialize(blocked);
    MotionSettings settings=blocked.settings();settings.set(MP_TRAVEL_PWM_MAX,50000);settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,50000);
    check(blocked.configure(settings) && blocked.commandPositioning(0,direction*10,500,500),"Blockadepruefung nutzt erhoehte Tilt-Anlaufreserve");
    uint32_t now=200;
    for(;now<3000 && !blocked.fault();now+=20) blocked.tick(now,0,0,true,true);
    check(blocked.fault() && std::strcmp(blocked.fault(),"MOTION_STALLED")==0 && now<=1700 &&
      blocked.status(true).signedPwm==0 && blocked.status(false).signedPwm==0,
      "Mehr Tilt-Reserve verlaengert keine echte Blockade und schaltet weiterhin beide Motoren ab");
  }
  return motionChecks-before;
}

int runWeakDriveRegressions()
{
  int before=motionChecks;
  // A real acquisition target, initially easy running, then increased load.
  // Keep the load below the existing PWM cap; only torque recovery is tested.
  for(int direction : {-1,1}) {
    MotionController motion; initialize(motion);
    MotionSettings settings=motion.settings(); settings.set(MP_PAN_JOG_START_PWM,50000);
    motion.configure(settings);
    check(motion.commandPositioning(direction*20.0,0,200,200),"Lastwechsel nutzt den echten Aufnahme-Fahrbefehl");
    double position=0,velocity=0;
    unsigned pause=0,longest=0; int peak=0;
    for(uint32_t now=200;now<30000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,direction*position,0,true,true);
      int pwm=motion.status(false).signedPwm*direction; peak=std::max(peak,pwm);
      double friction=position<5.0 ? 10.0 : 34.0;
      double breakaway=position<5.0 ? 20.0 : 45.0;
      if(velocity<0.05 && pwm<breakaway) velocity=0;
      else velocity=std::max(0.0,velocity+(pwm-friction-velocity/1.3)*0.02/0.18);
      position+=velocity*0.02;
      if(position>5 && position<19 && velocity<0.05) {pause+=20;longest=std::max(longest,pause);} else pause=0;
    }
    std::cout<<"Weak drive dir="<<direction<<" position="<<position<<" longestPauseMs="<<longest<<" peakPwm="<<peak
      <<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(longest<=1000,"Schwache Ansteuerung muss innerhalb 1000 ms Lastreserve aufbauen statt sekundenlang stehenzubleiben");
    check(!motion.fault() && motion.reached() && peak<=50,"Lastreserve erreicht das Ziel innerhalb bestehender Leistungsgrenze");
  }
  return motionChecks-before;
}

int runPanoramaNavigationRegressions()
{
  int before=motionChecks;
  for(int direction : {-1,1}) for(double distance : {3.0,9.0,13.0}) {
    MotionController motion; initialize(motion); MotionSettings config=motion.settings();
    config.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); config.set(MP_TRAVEL_DOWN_I_MAX,15000); motion.configure(config);
    motion.commandManual(0,direction*distance,500,200); FrictionTiltDrive drive(0,1.3,22,19,28);
    for(uint32_t now=200;now<24000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,0,drive.position,true,true); drive.step(motion.status(true).signedPwm);
    }
    std::cerr<<"Tilt navigation distance="<<distance<<" dir="<<direction<<" pos="<<drive.position<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(motion.reached() && !motion.fault() && std::abs(direction*distance-drive.position)<=0.3,
      "Tilt-Zielpunkte kurzer und laengerer Fahrten bleiben unter Last in Toleranz");
  }
  for(int direction : {-1,1}) for(double distance : {1.5,3.47,20.0}) {
    MotionController motion; initialize(motion);
    MotionSettings config=motion.settings(); config.set(MP_PAN_JOG_START_PWM,50000); motion.configure(config);
    motion.commandManual(direction*distance,0,500,200);
    FrictionTiltDrive drive(0,1.3,34,34,45);
    for(uint32_t now=200;now<18000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,drive.position,0,true,true); drive.step(motion.status(false).signedPwm);
    }
    std::cerr<<"Navigation distance="<<distance<<" dir="<<direction<<" pos="<<drive.position<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(motion.reached() && !motion.fault() && std::abs(direction*distance-drive.position)<=0.3,
      "Kurze Pan-Zielpunkte bei App-Tempo 500 bleiben trotz beladener Anlaufgeschwindigkeit in Toleranz");
  }
  return motionChecks-before;
}

int runDownBrakeRegression()
{
  int before=motionChecks;
  MotionController motion; initialize(motion); MotionSettings settings=motion.settings();
  settings.set(MP_TRAVEL_DOWN_BRAKE_LEAD_SECONDS,110); settings.set(MP_MANUAL_TILT_DOWN_COAST_DECELERATION,50000);
  motion.configure(settings); motion.commandPositioning(0,-3.083,1000,1000);
  double remaining=3.083;
  for(uint32_t now=200;now<1800 && !motion.fault();now+=20) {
    double position=-std::min(3.083,std::max(0.0,(double(now)-920)*.008));
    motion.tick(now,0,position,true,true);
    if(motion.status(true).driveStage==DRIVE_BRAKE) { remaining=3.083+position; break; }
  }
  std::cerr<<"Down brake remaining="<<remaining<<std::endl;
  check(!motion.fault() && remaining<1.0 && remaining>0,
    "Abwaertsbremse zaehlt dieselbe Nachlaufstrecke nicht zweimal im Anlauffenster");
  return motionChecks-before;
}

int runPanReturnLoadRegressions()
{
  int before=motionChecks;
  // Reduced hardware failure: a finite 2.062-degree Pan move begins normally,
  // then running load rises while still below the configured PWM ceiling.
  // The old finite PI stalls at 1.390 degrees and faults after 1500 ms.
  for(int direction : {-1,1}) for(int mode=0;mode<3;++mode) for(bool noisy : {false,true}) {
    MotionController motion; initialize(motion);
    MotionSettings settings=motion.settings();
    settings.set(MP_PAN_JOG_START_PWM,50000);
    settings.set(MP_PAN_TARGET_COAST_DECELERATION,100000);
    motion.configure(settings);
    check(mode==0 ? motion.commandManual(direction*2.062,0,1000,1000) :
      mode==1 ? motion.commandPositioning(direction*2.062,0,1000,1000) :
      motion.commandColumnReturn(direction*2.062,22,1000,1000),
      "Manuelles Ziel, Aufnahmeziel und Spaltenrueckfahrt nutzen den Pan-Lastausgleich");
    FrictionTiltDrive tilt(0,1.3,22,19,28);
    double position=0,velocity=0;
    double tolerance=mode==0 ? 0.3 : 0.15;
    int previousPwm=0,peak=0;
    bool boundedSlew=true,forward=true,requestedMotion=true;
    MotionTraceSnapshot last=motion.traceSnapshot(false);
    for(uint32_t now=200;now<10000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,direction*position+(noisy ? 0.022*std::sin(now*0.012) : 0),tilt.position,true,true);
      if(!motion.fault()) last=motion.traceSnapshot(false);
      int pwm=motion.status(false).signedPwm*direction;
      boundedSlew=boundedSlew && pwm-previousPwm<=1;
      forward=forward && pwm>=0;
      peak=std::max(peak,pwm); previousPwm=pwm;
      if(last.axis.driveStage==DRIVE_CRUISE && 2.062-position>tolerance)
        requestedMotion=requestedMotion && std::abs(last.axis.referenceVelocity)>0;
      double friction=position<1.3 ? 34.0 : 46.0;
      double breakaway=position<1.3 ? 45.0 : 49.0;
      if(std::abs(velocity)<0.05 && std::abs(pwm)<breakaway) velocity=0;
      else {
        double next=velocity+(pwm-friction-velocity/0.35)*0.02/0.18;
        velocity=std::max(0.0,next);
      }
      position+=velocity*0.02;
      tilt.step(motion.status(true).signedPwm);
    }
    if(!motion.reached() || motion.fault()) std::cerr<<"Pan return load direction="<<direction<<" mode="<<mode
      <<" noisy="<<noisy<<" pos="<<position<<" pwm="<<last.axis.signedPwm
      <<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(motion.reached() && !motion.fault() && std::abs(2.062-position)<=tolerance,
      "Pan-Spaltenrueckfahrt erreicht bei Lastanstieg das Ziel statt MOTION_STALLED");
    check(boundedSlew && forward && peak<=50 && requestedMotion && motion.status(false).signedPwm==0,
      "Lastausgleich erhaelt PWM-Grenze und sanften Aufbau ohne Umkehr oder vorzeitige Sollruhe");
  }
  for(bool recovery : {false,true}) {
    MotionController blocked; initialize(blocked);
    MotionSettings settings=blocked.settings(); settings.set(MP_PAN_JOG_START_PWM,50000);
    if(!recovery) settings.set(MP_PAN_TARGET_RECOVERY_I_GAIN,0);
    blocked.configure(settings); blocked.commandColumnReturn(3,22,1000,1000);
    FrictionTiltDrive tilt(0,1.3,22,19,28);
    uint32_t now=200;
    for(;now<5000 && !blocked.fault();now+=20) {
      blocked.tick(now,std::min(1.3,(now-200)*0.002),tilt.position,true,true);
      tilt.step(blocked.status(true).signedPwm);
    }
    check(blocked.fault() && std::strcmp(blocked.fault(),"MOTION_STALLED")==0 && now<=2650,
      "Echte Blockade nach Anlaufen endet auch mit Lastausgleich ohne verlaengerte Stillstandsfrist");
    check(blocked.status(false).signedPwm==0 && blocked.status(true).signedPwm==0,
      "Pan-Blockade schaltet bei gleichzeitiger Tilt-Rueckfahrt beide Motoren ab");
  }
  return motionChecks-before;
}

int runColumnArrivalRegressions()
{
  int before=motionChecks;
  // The user run reached column four with Pan resting 0.23 degrees beyond
  // its endpoint while Tilt still had about six degrees of return left.
  // Replay that finite arrival/coast pattern through the real controller.
  for(double error : {0.23,-0.23,0.31,0.45,-0.31,-0.472}) {
    MotionController motion; initialize(motion,0,-13.561);
    MotionSettings settings=motion.settings();
    settings.set(MP_PAN_JOG_START_PWM,50000);
    settings.set(MP_PAN_TARGET_COAST_DECELERATION,60000);
    // Calibrated rig profile: same arrival band as Tilt and manual Pan.
    // Production firmware already supports these user-adjustable settings.
    settings.set(MP_POSITION_ENTER_DEG,300);
    settings.set(MP_POSITION_EXIT_DEG,400);
    motion.configure(settings); motion.commandColumnReturn(2.062,8.020,1000,1000);
    FrictionTiltDrive tilt(-13.561,1.3,22,19,28);
    for(uint32_t now=200;now<12000 && !motion.fault() && !motion.reached();now+=20) {
      double pan=std::min(2.062+error,std::max(0.0,(double(now)-800)*0.008));
      motion.tick(now,pan,tilt.position,true,true);
      tilt.step(motion.status(true).signedPwm);
    }
    if(std::abs(error)<0.3) {
      if(motion.fault()) std::cerr<<"Column arrival error="<<error<<" Tilt="<<tilt.position
        <<" fault="<<motion.fault()<<std::endl;
      check(motion.reached() && !motion.fault() && std::abs(tilt.position-8.020)<=0.3,
        "Normale Pan-Ankunftsstreuung bricht die noch laufende Tilt-Spaltenrueckfahrt nicht ab");
    } else {
      check(motion.fault() && std::strcmp(motion.fault(),"MOTION_TARGET_MISSED")==0 &&
        motion.status(false).signedPwm==0 && motion.status(true).signedPwm==0,
        "Pan-Ankunft ausserhalb der festgelegten Toleranz stoppt weiterhin beide Motoren");
      check(std::abs(tilt.position-8.020)<=0.3,
        "Pan-Zielwarnung darf die noch weit vom Ziel entfernte Tilt-Rueckfahrt nicht abbrechen");
    }
  }
  return motionChecks-before;
}

int runConcurrentArrivalSafetyRegressions()
{
  int before=motionChecks;
  for(int action=0;action<3;++action) {
    MotionController motion;initialize(motion,0,-13.561);MotionSettings settings=motion.settings();
    settings.set(MP_PAN_JOG_START_PWM,50000);settings.set(MP_PAN_TARGET_COAST_DECELERATION,60000);
    settings.set(MP_POSITION_ENTER_DEG,300);settings.set(MP_POSITION_EXIT_DEG,400);
    check(motion.configure(settings) && motion.commandColumnReturn(2.062,8.020,1000,1000),"Gekoppelte Schutzpruefung startet echte Spaltenrueckfahrt");
    FrictionTiltDrive tilt(-13.561,1.3,22,19,28);uint32_t now=200;bool pending=false;
    for(;now<6000 && !motion.fault();now+=20) {
      double pan=std::min(2.512,std::max(0.0,(double(now)-800)*0.008));
      motion.tick(now,pan,tilt.position,true,true);tilt.step(motion.status(true).signedPwm);
      if(motion.status(false).driveStage==DRIVE_SETTLE && !motion.status(false).withinTolerance) {pending=true;break;}
    }
    check(pending && !motion.fault() && motion.status(false).signedPwm==0 && tilt.position<7.7,
      "Verfehlte Pan-Achse bleibt stromlos, waehrend die andere Achse noch fahren darf");
    if(action==0) {
      motion.tick(now+20,2.512,tilt.position,false,true);
      check(motion.fault() && std::strcmp(motion.fault(),"MOTION_SENSOR")==0 &&
        motion.status(false).signedPwm==0 && motion.status(true).signedPwm==0,
        "Sensorfehler stoppt trotz aufgeschobener Zielwarnung sofort beide Achsen");
    } else if(action==1) {
      for(uint32_t t=now+20;t<now+5000 && !motion.fault();t+=20) motion.tick(t,2.512,tilt.position,true,true);
      check(motion.fault() && std::strcmp(motion.fault(),"MOTION_STALLED")==0 &&
        motion.status(false).signedPwm==0 && motion.status(true).signedPwm==0,
        "Blockierte zweite Achse behaelt den normalen Stillstandsschutz");
    } else {
      motion.tick(now+20,2.512,tilt.position,true,false);
      check(!motion.fault() && motion.status(false).signedPwm==0 && motion.status(true).signedPwm==0,
        "Entzogene Fahrfreigabe verwirft auch die aufgeschobene Zielwarnung");
      motion.resetFeedback();initialize(motion);
      check(motion.commandColumnReturn(0,2,500,500),"Neuer Auftrag nach Stop wird ohne alten Warnzustand angenommen");
    }
  }
  {
    MotionController motion;initialize(motion);MotionSettings settings=motion.settings();
    settings.set(MP_TRAVEL_POSITION_ENTER_DEG,300);settings.set(MP_TRAVEL_POSITION_EXIT_DEG,400);
    motion.configure(settings);motion.commandColumnReturn(20,3,1000,1000);
    bool pending=false,panDrivenAfterMiss=false;
    for(uint32_t now=200;now<30000 && !motion.fault();now+=20) {
      double pan=std::min(20.0,(now-200)*.001);
      double tilt=std::min(3.4,(now-200)*.006);
      motion.tick(now,pan,tilt,true,true);
      if(motion.status(true).driveStage==DRIVE_SETTLE && !motion.status(true).withinTolerance) {
        pending=true;panDrivenAfterMiss=panDrivenAfterMiss || motion.status(false).signedPwm>0;
        check(motion.status(true).signedPwm==0,"Zuerst verfehlte Tilt-Achse erhaelt keine weitere Leistung");
      }
      if(motion.fault()) check(pan>=19.7,"Tilt-Zielwarnung darf die noch laufende Pan-Fahrt nicht vorzeitig abbrechen");
    }
    check(pending && panDrivenAfterMiss && motion.fault() && std::strcmp(motion.fault(),"MOTION_TARGET_MISSED")==0 &&
      std::abs(motion.status(false).position-20)<=.3,"Auch bei vertauschten Achsen wird erst nach Abschluss der anderen Fahrt gewarnt");
  }
  return motionChecks-before;
}

int runPanArrivalProgressRegression()
{
  int before=motionChecks;
  MotionController motion;initialize(motion);
  MotionSettings settings=motion.settings();settings.set(MP_POSITION_ENTER_DEG,300);settings.set(MP_POSITION_EXIT_DEG,400);
  motion.configure(settings);motion.commandPositioning(3,0,1000,1000);
  for(uint32_t now=200;now<5000 && !motion.fault();now+=20)
    motion.tick(now,std::min(2.65,(now-200)*.001),0,true,true);
  check(motion.fault() && std::strcmp(motion.fault(),"MOTION_STALLED")==0 && motion.status(false).signedPwm==0,
    "Stillstand ausserhalb 0.30 Grad bleibt auch innerhalb 0.40 Grad Haltehysterese zeitlich ueberwacht");
  MotionController within;initialize(within);within.configure(settings);within.commandPositioning(3,0,1000,1000);
  for(uint32_t now=200;now<5000 && !within.fault() && !within.reached();now+=20)
    within.tick(now,std::min(2.75,(now-200)*.001),0,true,true);
  check(within.reached() && !within.fault() && within.status(false).signedPwm==0,
    "Bereits ruhige Achse innerhalb der Ankunftstoleranz kriecht nicht unbegrenzt weiter");
  return motionChecks-before;
}

int runPanStopModelRegressions()
{
  int before=motionChecks;
  for(const PanFitFixture &fixture : panFitFixtures) {
    double position=0,velocity=0;
    check(PanStopModel::estimate(fixture.time,fixture.position,5,position,velocity),
      "Bremsmodell akzeptiert echte ungleichmaessig zeitgestempelte Messproben");
    check(std::abs(position-fixture.expectedPosition)<1e-7 && std::abs(velocity-fixture.expectedVelocity)<1e-6,
      "Firmware-Bremsmodell stimmt mit unabhaengiger NumPy-Anpassung echter Fahrdaten ueberein");
  }
  double times[5]={0,0,0,0,0},positions[5]={1,1,1,1,1},position=0,velocity=0;
  check(!PanStopModel::estimate(times,positions,5,position,velocity),"Ungueltige Zeitbasis benutzt kein singulaeres Bremsmodell");
  for(unsigned int i=0;i<5;++i) {times[i]=((double)i-4)*.02;positions[i]=120+8*times[i]+20*times[i]*times[i];}
  check(PanStopModel::estimate(times,positions,5,position,velocity) && std::abs(position-120)<1e-8 && std::abs(velocity-8)<1e-7,
    "Beschleunigung wird auf aktuellen Zeitpunkt bezogen statt als verzoegerter Mittelwert verwendet");
  double cutoff[2]={0,0};
  for(unsigned int calibrated=0;calibrated<2;++calibrated) {
    MotionController motion;initialize(motion);MotionSettings settings=motion.settings();
    settings.set(MP_POSITION_ENTER_DEG,300);settings.set(MP_POSITION_EXIT_DEG,400);
    settings.set(MP_PAN_JOG_START_PWM,50000);settings.set(MP_PAN_TARGET_COAST_DECELERATION,100000);
    settings.set(MP_PAN_TARGET_PREDICTION_SAMPLES,calibrated ? 5 : 0);
    motion.configure(settings);motion.commandPositioning(5,0,1000,1000);
    for(uint32_t now=200;now<3000 && !motion.fault();now+=20) {
      double elapsed=(now-200)/1000.0;double actual=2*elapsed*elapsed;
      motion.tick(now,actual,0,true,true);
      if(motion.status(false).driveStage==DRIVE_BRAKE) { cutoff[calibrated]=actual;break; }
    }
  }
  check(cutoff[0]>0 && cutoff[1]>0 && cutoff[1]<cutoff[0]-.05,
    "Aktiviertes Bremsmodell schaltet beschleunigende Achse vor der verzoegerten alten Vorhersage ab");
  return motionChecks-before;
}

int runBufferedPanTraceTests()
{
  int before=motionChecks;
  BufferedPanTrace trace;
  trace.sample(0,1000,0,false,true);
  check(!trace.ready() && trace.count()==0,"Ruhende Achse startet keinen Messpuffer");
  trace.sample(20,1000,1,true,true);trace.sample(40,1022,2,true,true);
  trace.sample(60,1044,0,false,true);
  check(trace.ready() && trace.count()==4 && trace.at(0).pwm==0 && trace.at(1).deltaMs==20 && trace.at(3).positionMdeg==1044,
    "Messpuffer umfasst Vorprobe, echte Stellwerte und bestaetigten Abschaltzeitpunkt");
  uint16_t crc=trace.checksum();
  trace.sample(80,1088,0,false,true);
  check(trace.count()==4 && trace.checksum()==crc && trace.at(3).positionMdeg==1044,
    "Nach Abschaltung bleibt die Messung unveraendert fuer spaeteres Auslesen erhalten");
  trace.sample(100,1088,-1,true,true);trace.sample(120,1066,0,false,false);
  check(trace.ready() && trace.count()==3 && trace.at(0).positionMdeg==1088 && trace.at(1).pwm==-1 && trace.invalid()==1,
    "Neue Fahrt beginnt neue Messung; ungueltige Sensorwerte bleiben sichtbar");
  BufferedPanTrace longRun;
  longRun.sample(0,0,0,false,true);
  for(unsigned int i=1;i<=BufferedPanTrace::CAPACITY+5;++i) longRun.sample(i*20,i,1,true,true);
  longRun.sample((BufferedPanTrace::CAPACITY+6)*20,9999,0,false,true);
  check(longRun.ready() && longRun.count()==BufferedPanTrace::CAPACITY && longRun.overwritten()==7 &&
    longRun.at(longRun.count()-1).positionMdeg==9999,
    "Zu lange Messung meldet ueberschriebene Daten statt unvollstaendige Identifikation vorzutaueschen");
  BufferedPanTrace gap;gap.sample(0,0,0,false,true);gap.sample(200,0,1,true,true);gap.sample(220,0,0,false,true);
  check(gap.badTiming(),"Messpuffer kennzeichnet zu grosse Zeitluecken");
  return motionChecks-before;
}

int runPanoramaStartRegressions()
{
  int before=motionChecks;
  uint32_t returns[2]={0,0};
  for(int fast=0;fast<2;++fast) {
    MotionController motion; initialize(motion,0,-20); MotionSettings settings=motion.settings();
    settings.set(MP_COLUMN_RETURN_VELOCITY,fast ? 8000 : 0); motion.configure(settings);
    motion.commandColumnReturn(0,2,500,800); FrictionTiltDrive drive(-20,1.3,22,19,28);
    double maximumReference=0;
    for(uint32_t now=200;now<16000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,0,drive.position,true,true); drive.step(motion.status(true).signedPwm);
      maximumReference=std::max(maximumReference,motion.status(true).referenceVelocity);
      if(motion.reached()) returns[fast]=now-180;
    }
    check(motion.reached() && !motion.fault() && maximumReference<=(fast ? 8.0 : 4.0)+1e-8,
      "Rueckfahrt verwendet eigene Geschwindigkeitsgrenze und endet innerhalb der Tilt-Toleranz");
    std::cerr<<"Column return fast="<<fast<<" duration="<<returns[fast]<<" maxReference="<<maximumReference<<std::endl;
    motion.disable(); for(uint32_t now=16200;now<=16500;now+=20) motion.tick(now,0,drive.position,true,false);
    motion.commandPositioning(0,drive.position-3,500,500); motion.tick(16520,0,drive.position,true,true);
    check(std::abs(motion.status(true).referenceAcceleration)<=settings.number(MP_JOG_ACCELERATION)+1e-8,
      "Folgender Aufnahmeschritt erbt keine schnelle Rueckfahrt-Beschleunigung");
  }
  check(returns[1]>0 && returns[1]<=returns[0]*0.65,"22-Grad-Spaltenrueckfahrt wird im Lastmodell deutlich schneller");
  for(int direction : {-1,1}) {
    MotionController motion; initialize(motion); MotionSettings config=motion.settings();
    config.set(MP_PAN_JOG_START_PWM,50000); motion.configure(config);
    motion.commandPositioning(direction*20.0,0,1000,1000); FrictionTiltDrive drive(0,1.3,34,34,45);
    bool moving=false; unsigned pause=0,longest=0;
    for(uint32_t now=200;now<18000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,drive.position,0,true,true); drive.step(motion.status(false).signedPwm);
      if(drive.velocity*direction>0.5) moving=true;
      if(moving && std::abs(direction*20-drive.position)>1 && std::abs(drive.velocity)<0.5) { pause+=20; longest=std::max(longest,pause); } else pause=0;
    }
    std::cerr<<"Run Pan start dir="<<direction<<" pause="<<longest<<" pos="<<drive.position<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(moving && !motion.fault() && motion.reached() && longest<=100,
      "Pan-Startposition im Lauf bleibt unter Last kontinuierlich ohne erneutes Losbrechen");
  }
  for(int direction : {-1,1}) for(bool noisy : {false,true}) {
    MotionController motion; initialize(motion);
    motion.commandPositioning(0,direction*10.0,1000,1000); FrictionTiltDrive drive(0,1.3,22,19,28);
    bool moving=false; unsigned pause=0,longest=0;
    for(uint32_t now=200;now<8000 && !motion.fault() && !motion.reached();now+=20) {
      double noise[]={-0.066,0.044,0.088,-0.022,0.066,-0.088,0.022,-0.044,0,0.044};
      double encoder=drive.position+(noisy && motion.status(true).signedPwm!=0 ? noise[(now/20)%10] : 0);
      motion.tick(now,0,encoder,true,true); drive.step(motion.status(true).signedPwm);
      if(drive.velocity*direction>0.5) moving=true;
      if(moving && std::abs(direction*10-drive.position)>1 && std::abs(drive.velocity)<1.0) { pause+=20; longest=std::max(longest,pause); } else pause=0;
    }
    std::cerr<<"Run start dir="<<direction<<" pause="<<longest<<" pos="<<drive.position<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(moving && !motion.fault() && motion.reached() && longest<=100,
      "Startposition des Laufs uebernimmt Anlaufleistung ohne Stocken");
  }
  return motionChecks-before;
}

int runMotionTests()
{
  {
    MotionController fast;initialize(fast);MotionSettings config=fast.settings();
    config.values[MP_MOVE_AUTO_DURATION]=1;config.values[MP_JOG_ACCELERATION]=2000;
    check(fast.configure(config),"Tempo-Erweiterung mit Zeitmodell konfiguriert");
    check(fast.commandPositioning(0,-32.582,1000,1000),"Bisheriges Volltempo weiterhin angenommen");
    uint32_t oldBudget=fast.plannedMoveMaximumDurationMs();fast.disable();initialize(fast);
    check(fast.commandPositioning(0,-32.582,2000,2000) && fast.plannedMoveMaximumDurationMs()<oldBudget,
      "Doppeltes Lauftempo verkürzt physikalisches Budget bei ausreichender Strecke");
    check(fast.targetVelocity(true,2000)==2*config.axis(true).maximumVelocity,"2000 Promille verdoppelt Solltempo");
    fast.disable();initialize(fast);config.values[MP_RUN_TILT_MAX_VELOCITY]=6000;
    check(fast.configure(config) && fast.targetVelocity(true,2000)==6 && fast.targetVelocity(true,1000)==5,
      "Absolute separate Laufgrenze begrenzt erhöhtes Tempo; 1000 bleibt unverändert");
    check(!fast.commandPositioning(0,-10,2001,1000),"Lauftempo über 2000 wird vor Bewegung abgelehnt");
  }
  int initialMotionCheckCount=motionChecks;
  testManualPanSteadyVelocity();
  for(int direction : {-1,1}) {
    MotionController motion; initialize(motion);
    MotionSettings config=motion.settings(); config.set(MP_PAN_JOG_START_PWM,50000); motion.configure(config);
    motion.commandManual(direction*10.0,0,200,200);
    bool sufficient=false;
    for(uint32_t now=200;now<=1200 && !motion.fault();now+=20) {
      // A startup encoder offset inside the quiet span must not falsely
      // confirm breakaway and replace the available PWM 50 by PWM 25.
      motion.tick(now,direction*(0.10+0.02*std::sin(now*0.01)),0,true,true);
      sufficient=sufficient || direction*motion.status(false).signedPwm>=45;
    }
    check(sufficient && !motion.fault(),"Encoder-Ruhebewegung bestaetigt keinen Pan-Anlauf mit zu kleiner Leistung");
  }
  for(int direction : {-1,1}) {
    MotionController overrun; initialize(overrun); overrun.commandManual(direction*3,0,200,200);
    for(uint32_t now=200;now<3500 && !overrun.fault();now+=20)
      overrun.tick(now,direction*std::min(3.6,(now-200)*0.008),0,true,true);
    check(overrun.fault() && std::strcmp(overrun.fault(),"MOTION_TARGET_MISSED")==0 && overrun.status(false).signedPwm==0,
      "Verfehltes ruhiges Pan-Ziel wird zeitnah erkannt statt minutenlang ohne Bewegung zu warten");
  }
  testManualPanLoadedStart();
  testManualPanLoadedArrival();
  for(int direction : {-1,1}) {
    MotionController blocked; initialize(blocked); blocked.commandManual(direction*3.0,0,200,200);
    for(uint32_t now=200;now<2500 && !blocked.fault();now+=20) blocked.tick(now,0,0,true,true);
    check(blocked.fault() && blocked.status(false).signedPwm==0 && blocked.status(true).signedPwm==0,
      "Tatsaechlich blockierte Pan-Zielfahrt behaelt Anlaufzeitgrenze und sichere Abschaltung");
    MotionController near; initialize(near); near.commandManual(direction*0.29,0,200,200);
    bool unpowered=true;
    for(uint32_t now=200;now<1500;now+=20) { near.tick(now,0,0,true,true); unpowered=unpowered && near.status(false).signedPwm==0; }
    check(near.reached() && !near.fault() && unpowered,
      "Pan-Ziel innerhalb der manuellen Toleranz erzeugt keinen Losbrechimpuls");
  }
  {
    double remaining[2]={0,0};
    for(int manual=0;manual<2;++manual) {
      MotionController motion; initialize(motion);
      MotionSettings config=motion.settings(); config.set(MP_MANUAL_TILT_DOWN_COAST_DECELERATION,10000); motion.configure(config);
      if(manual) motion.commandManual(0,-3,200,200); else motion.command(0,-3,200,200);
      for(uint32_t now=200;now<1000 && !motion.fault();now+=20) {
        double position=-double(now-200)*0.008; motion.tick(now,0,position,true,true);
        if(motion.status(true).driveStage==DRIVE_BRAKE) { remaining[manual]=3+position; break; }
      }
      check(remaining[manual]>0 && !motion.fault(),"Schneller Abwaertsanlauf hat einen gemessenen Bremspunkt");
    }
    check(remaining[1]>remaining[0]+0.3,"Konfigurierbare Nachlaufvorhersage finiter Abwaertsziele wirkt ohne das alte Profil zu veraendern");
  }

  for(int direction : {-1,1}) for(bool noisy : {false,true}) {
    MotionController motion; initialize(motion); MotionSettings config=motion.settings(); config.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); config.set(MP_TRAVEL_DOWN_I_MAX,15000); motion.configure(config);
    motion.commandManual(0,direction*3.0,200,200); FrictionTiltDrive drive(0,1.3,22,19,28);
    bool moving=false; unsigned pause=0,longest=0;
    for(uint32_t now=200;now<15000 && !motion.fault() && !motion.reached();now+=20) {
      double noise[]={-0.066,0.044,0.088,-0.022,0.066,-0.088,0.022,-0.044,0,0.044};
      double encoder=drive.position+(noisy && motion.status(true).signedPwm!=0 ? noise[(now/20)%10] : 0);
      motion.tick(now,0,encoder,true,true); drive.step(motion.status(true).signedPwm);
      if(drive.velocity*direction>0.5) moving=true;
      if(moving && std::abs(direction*3-drive.position)>0.3 && std::abs(drive.velocity)<0.05) { pause+=20; longest=std::max(longest,pause); } else pause=0;
    }
    std::cerr<<"Manual tilt dir="<<direction<<" pause="<<longest<<" pos="<<drive.position<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(motion.reached() && !motion.fault() && longest<=100,"Manuelle beladene Tilt-Zielfahrt stockt nach dem Anlauf nicht");
  }

  for(int direction : {-1,1}) {
    MotionController motion; initialize(motion); motion.commandManual(direction*10,0,200,200);
    for(uint32_t now=200;now<=2500;now+=20) motion.tick(now,direction*fmin(9.77,(now-200)*0.01),0,true,true);
    check(motion.reached(),"Manueller Toleranzfall erreicht");
    motion.command(direction*10,0,200,200);
    motion.tick(2520,direction*9.77,0,true,true);
    check(!motion.reached(),"Folgender Aufnahmeschritt gleicher Zielkoordinaten erbt keine manuelle Toleranz");
    MotionController strict; initialize(strict); MotionSettings config=strict.settings(); config.set(MP_MANUAL_PAN_TOLERANCE,200); strict.configure(config);
    strict.commandManual(direction*10,0,200,200);
    for(uint32_t now=200;now<=2500 && !strict.fault();now+=20) strict.tick(now,direction*fmin(9.77,(now-200)*0.01),0,true,true);
    check(!strict.reached(),"User-Toleranz 0.2 Grad akzeptiert den Restfehler 0.23 nicht");
  }

  for(int direction : {-1,1}) {
    MotionController motion; initialize(motion); motion.commandManual(direction*10,0,200,200);
    for(uint32_t now=200;now<=2500 && !motion.fault();now+=20)
      motion.tick(now,direction*fmin(9.77,(now-200)*0.01),0,true,true);
    check(motion.reached() && !motion.fault() && motion.status(false).signedPwm==0,
      "Manuelle Panposition mit 0.23 Grad Restfehler ruht ohne weitere Ruckkorrektur");
  }

  for(int direction : {-1,1}) {
    MotionController motion; initialize(motion);
    motion.command(direction*10.0,0,200,200);
    for(uint32_t now=200;now<=2500 && !motion.fault();now+=20)
      motion.tick(now,direction*fmin(9.90,(now-200)*0.01),0,true,true);
    check(motion.reached() && !motion.fault() && motion.status(false).signedPwm==0,
      "Bereits erreichte ruhige Panposition wird ohne Warten auf das langsame Sollprofil bestaetigt");
  }

  for(int direction : {-1,1}) {
    MotionController motion; initialize(motion);
    check(motion.command(direction*3.0,0,200,200),"Langsame Pan-Endannaherung planbar");
    double position=0,velocity=0; bool stuck=false; unsigned nearMs=0;
    for(uint32_t now=200;now<12000 && !motion.fault() && !motion.reached();now+=20) {
      if(!stuck) { position=std::min(2.64,std::abs(motion.status(false).reference)); stuck=position>=2.64; }
      motion.tick(now,direction*position,0,true,true);
      if(stuck) {
        int pwm=direction*motion.status(false).signedPwm;
        if(std::abs(velocity)<0.05 && std::abs(pwm)<34) velocity=0;
        else { double requested=pwm>=0 ? std::max(0.0,(pwm-18)*0.15) : std::min(0.0,(pwm+18)*0.15); velocity+=(requested-velocity)*0.02/0.10; }
        position+=velocity*0.02;
        if(std::abs(3-position)>0.25) nearMs+=20;
      }
    }
    std::cerr<<"Target regression dir="<<direction<<" near="<<nearMs<<" pos="<<position<<" fault="<<(motion.fault()?motion.fault():"none")<<std::endl;
    check(motion.reached() && !motion.fault() && nearMs<=1500,"Pan-Endannaherung bleibt auch bei Tempo 200 zeitlich begrenzt");
  }
  {
    MotionController jog,normal; initialize(jog); initialize(normal);
    jog.commandJogVelocity(true,1,5.0); jog.tick(200,0,0,true,true); jog.disable();
    for(uint32_t now=220;now<=500;now+=20) { jog.tick(now,0,0,true,false); normal.tick(now,0,0,true,false); }
    check(jog.command(0,2,500,500) && normal.command(0,2,500,500),"Normale Zielbewegung nach absolutem Halten startet");
    bool unchanged=true;
    for(uint32_t now=520;now<1400;now+=20) {
      jog.tick(now,0,0,true,true); normal.tick(now,0,0,true,true);
      unchanged=unchanged && jog.status(true).signedPwm==normal.status(true).signedPwm && jog.status(true).driveStage==normal.status(true).driveStage;
    }
    check(unchanged,"Absoluter Haltemodus wirkt nicht auf die folgende normale Panorama-Zielfahrt");
  }
  for(bool tilt : {false,true}) for(int direction : {-1,1}) for(bool noisy : {false,true}) {
    MotionController jog; initialize(jog); jog.configureLimits(-180,180,-60,60);
    MotionSettings settings=jog.settings(); settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); settings.set(MP_TRAVEL_DOWN_I_MAX,15000); settings.set(MP_JOG_ACCELERATION,1000); jog.configure(settings);
    double requested=tilt?5.0:10.0;
    check(jog.commandJogVelocity(tilt,direction,requested),"Absolute Haltefahrt fordert Tilt 5 / Pan 10 Grad/s");
    FrictionTiltDrive drive(0,1.3,tilt?22.0:20.0,tilt?19.0:21.0,tilt?28.0:32.0);
    const double noise[]={-0.066,0.044,0.088,-0.022,0.066,-0.088,0.022,-0.044,0.0,0.044};
    bool moving=false; unsigned pause=0,longestPause=0,count=0; double sum=0,ref=0; bool ramp=true;
    for(uint32_t now=200;now<11000 && !jog.fault();now+=20) {
      double position=drive.position+(noisy?noise[(now/20)%10]:0);
      jog.tick(now,tilt?0:position,tilt?position:0,true,true);
      AxisMotionStatus s=jog.status(tilt); drive.step(s.signedPwm);
      ramp=ramp && std::abs(s.referenceVelocity)<=requested+0.00001 && std::abs(s.referenceVelocity)-ref<=settings.number(tilt?MP_JOG_ACCELERATION:MP_PAN_JOG_ACCELERATION)*0.020+0.00001;
      ref=std::abs(s.referenceVelocity);
      if(drive.velocity*direction>0.5) moving=true;
      if(moving && drive.velocity*direction<0.05) { pause+=20; longestPause=std::max(longestPause,pause); } else pause=0;
      if(now>=8000) { sum+=drive.velocity*direction; ++count; }
    }
    if(jog.fault() || longestPause>100 || !count || sum/count<requested*0.88 || sum/count>requested*1.12)
      std::cerr<<"Absolute jog tilt="<<tilt<<" dir="<<direction<<" noisy="<<noisy<<" pause="<<longestPause<<" mean="<<(count?sum/count:0)<<" fault="<<(jog.fault()?jog.fault():"none")<<std::endl;
    check(ramp && moving && !jog.fault() && longestPause<=100 && count>0 && sum/count>=requested*0.88 && sum/count<=requested*1.12,
      "Haltefahrt startet begrenzt, bleibt kontinuierlich und erreicht Tilt 5 / Pan 10 Grad/s auch mit Encoderrauschen");
    jog.disable(); check(jog.status(false).signedPwm==0 && jog.status(true).signedPwm==0,"Absolute Haltefahrt stoppt beide Ausgaenge");
  }
  {
    MotionController jog; initialize(jog);
    MotionSettings settings=jog.settings(); settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); settings.set(MP_TRAVEL_DOWN_I_MAX,15000); jog.configure(settings);
    jog.commandJogVelocity(true,1,5.0); FrictionTiltDrive drive(0,1.3,22.0,19.0,28.0);
    bool moving=false; unsigned pause=0, longestPause=0;
    for(uint32_t now=200;now<4500 && !jog.fault();now+=20) {
      jog.tick(now,0,drive.position,true,true); drive.step(jog.status(true).signedPwm);
      if(drive.velocity>0.5) moving=true;
      if(moving && drive.velocity<0.05) { pause+=20; longestPause=std::max(longestPause,pause); } else pause=0;
    }
    if(!moving || jog.fault() || longestPause>100) std::cerr<<"Held startup: longest pause="<<longestPause<<" ms, fault="<<(jog.fault()?jog.fault():"none")<<std::endl;
    check(moving && !jog.fault() && longestPause<=100,
      "Gehaltene Taste behaelt nach dem Losbrechen eine kontinuierliche Fahrt ohne neue Haftreibungspause");
  }
  {
    MotionController jog; initialize(jog); MotionSettings settings=jog.settings();
    settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); settings.set(MP_TRAVEL_DOWN_I_MAX,15000); jog.configure(settings);
    jog.commandJog(true,-1,200); FrictionTiltDrive drive(0,1.3,22.0,37.0);
    const double noise[]={-0.066,0.044,0.088,-0.022,0.066,-0.088,0.022,-0.044,0.0,0.044};
    for(uint32_t now=200;now<9000 && !jog.fault();now+=20) {
      jog.tick(now,0,drive.position+noise[(now/20)%10],true,true); drive.step(jog.status(true).signedPwm);

    }
    if(jog.fault() || drive.position>=-1.0) std::cerr<<"Noisy jog position="<<drive.position<<" fault="<<(jog.fault()?jog.fault():"none")<<std::endl;
    check(!jog.fault() && drive.position<-1.0,
      "Beladenes Abwaerts-Jog bleibt trotz Encoderrauschen kraeftig und ohne Fehl-Stillstand");
  }
  {
    MotionController jog; initialize(jog); MotionSettings settings=jog.settings();
    settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); settings.set(MP_TRAVEL_DOWN_I_MAX,15000); jog.configure(settings);
    jog.commandJog(true,-1,200); FrictionTiltDrive drive(0,1.3,22.0,37.0);
    double peak=0, speed=0; unsigned count=0;
    for(uint32_t now=200;now<9000 && !jog.fault();now+=20) {
      if(now>=2500) drive.negativeFriction=19.0;
      jog.tick(now,0,drive.position,true,true); drive.step(jog.status(true).signedPwm);
      if(now>=2500) peak=std::max(peak,-drive.velocity);
      if(now>=5000) { speed-=drive.velocity; ++count; }
    }
    if(jog.fault() || peak>=7.0 || !count || speed/count>1.5)
      std::cerr<<"Jog friction release peak="<<peak<<" mean="<<(count?speed/count:0)<<std::endl;
    check(!jog.fault() && peak<7.0 && count>0 && speed/count>=0.6 && speed/count<=1.5,
      "Nach abfallender Last wird Jog-Losbrechreserve entfernt und moderates Lauftempo wiederhergestellt");
  }
  for(int direction : {-1,1}) {
    MotionController pan; initialize(pan); pan.commandJog(false,direction,200);
    FrictionTiltDrive drive(0,1.3,20.0,21.0,32.0);
    double speed=0, peak=0; unsigned count=0; bool bounded=true;
    for(uint32_t now=200;now<8000 && !pan.fault();now+=20) {
      pan.tick(now,drive.position,0,true,true); int pwm=pan.status(false).signedPwm;
      bounded=bounded && std::abs(pwm)<=40 && pwm*direction>=0 && pan.status(true).signedPwm==0;
      drive.step(pwm); peak=std::max(peak,direction*drive.velocity);
      if(now>=2500) { speed+=direction*drive.velocity; ++count; }
    }
    if(!bounded || pan.fault() || count==0 || speed/count<1.1 || speed/count>2.1 || peak>=4.0)
      std::cerr<<"Pan jog dir="<<direction<<" mean="<<(count ? speed/count : 0)<<" peak="<<peak<<" fault="<<(pan.fault()?pan.fault():"none")<<std::endl;
    check(bounded && !pan.fault() && count>0 && speed/count>=1.1 && speed/count<=2.1 && peak<4.0,
      "Pan-Jog regelt reales moderates Tempo ohne Positionsintegral und hohe Anlaufspitzen");
    pan.disable(); check(pan.status(false).signedPwm==0,"Pan-Jog Loslassen schaltet ohne Nachlaufziel ab");
  }
  for(int direction : {-1,1}) {
    MotionController pan; initialize(pan);
    check(pan.commandJog(false,direction,200),"Langsames Pan-Jog aus bestaetigtem Stillstand beginnt");
    bool torque=false;
    for(uint32_t now=200;now<1700 && !pan.fault();now+=20) {
      pan.tick(now,0,0,true,true);
      if(pan.status(false).signedPwm*direction>=35) { torque=true; break; }
    }
    check(torque && !pan.fault(),"Stiller Pan-Anlauf kann trotz langsamem Solltempo notwendige Losbrechkraft aufbauen");
  }
  {
    MotionController jog; initialize(jog);
    check(jog.commandJog(true,-1,300),"Gedrueckthalten startet eine Tilt-Dauerfahrt");
    double previous=0;
    for(uint32_t now=200;now<800;now+=20) {
      jog.tick(now,0,0,true,true);
      AxisMotionStatus s=jog.status(true);
      check(std::abs(s.referenceVelocity)>=previous && std::abs(s.referenceVelocity)<=1.5 &&
        std::abs(s.referenceVelocity)-previous<=0.016,"Jog-Solltempo steigt langsam und bleibt begrenzt");
      previous=std::abs(s.referenceVelocity);
    }
    jog.disable(); jog.tick(800,0,0,true,true);
    check(jog.status(true).signedPwm==0 && jog.status(false).signedPwm==0,"Loslassen beendet Dauerfahrt ohne weitere Ziele");
  }
  testFineApproachThresholdsAndBudgets();
  testDownwardNoiseTorqueBound();
  testConfigurableLoadedDownwardStart();
  for(int direction : {-1,1}) {
    MotionController jog; initialize(jog);
    MotionSettings settings=jog.settings(); settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); settings.set(MP_TRAVEL_DOWN_I_MAX,15000);
    jog.configure(settings); check(jog.commandJog(true,direction,200),"Langsames Jog im schnellen Lastmodell beginnt");
    FrictionTiltDrive drive(0,1.3,8.5,19.0,28.0);
    double speed=0; unsigned count=0; bool bounded=true;
    for(uint32_t now=200;now<8000 && !jog.fault();now+=20) {
      jog.tick(now,0,drive.position,true,true); int pwm=jog.status(true).signedPwm;
      bounded=bounded && std::abs(pwm)<=40 && pwm*direction>=0 && jog.status(false).signedPwm==0;
      drive.step(pwm);
      if(now>=2500) { speed+=direction*drive.velocity; ++count; }
    }
    if(jog.fault() || count==0 || speed/count<0.6 || speed/count>1.4)
      std::cerr<<"Jog speed dir="<<direction<<" mean="<<(count ? speed/count : 0)<<" fault="<<(jog.fault()?jog.fault():"none")<<std::endl;
    check(bounded && !jog.fault() && count>0 && speed/count>=0.6 && speed/count<=1.4,
      "Jog kann trotz schneller Reaktion unter die alte Mindestleistung reduzieren und das moderate Solltempo regeln");
    jog.disable(); check(jog.status(true).signedPwm==0,"Modellfahrt stoppt sofort ohne neue Zielschritte");
  }
  {
    MotionController jog; initialize(jog); MotionSettings settings=jog.settings();
    settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000); settings.set(MP_TRAVEL_DOWN_I_MAX,15000); jog.configure(settings);
    jog.commandJog(true,-1,200); FrictionTiltDrive drive(0,1.3,22.0,37.0);
    for(uint32_t now=200;now<6000 && !jog.fault();now+=20) { jog.tick(now,0,drive.position,true,true); drive.step(jog.status(true).signedPwm); }
    check(!jog.fault() && drive.position<-1.0,"Jog behaelt starke Abwaertsreserve bei hoher Laufreibung ohne neue Anlaufserie");
  }
  {
    MotionController motion; initialize(motion);
    MotionSettings settings=motion.settings();
    check(settings.set(MP_TRAVEL_DOWN_I_MAX,15000),"Abwaertsregelung besitzt konfigurierbare Reserve bis zur Gesamtgrenze");
    settings.set(MP_TRAVEL_DOWN_START_PWM_MAX,40000);
    check(motion.configure(settings) && motion.command(0,-4,500,500),"Beladene Abwaertsfahrt mit hoher Laufreibung startet");
    FrictionTiltDrive drive(0,1.3,22.0,37.0);
    double totalSpeed=0; unsigned count=0;
    for(uint32_t now=200;now<16000 && !motion.fault() && !motion.reached();now+=20) {
      motion.tick(now,0,drive.position,true,true); drive.step(motion.status(true).signedPwm);
      if(drive.position<-0.5 && drive.position>-3.5) { totalSpeed-=drive.velocity; ++count; }
    }
    check(count>0 && totalSpeed/count>1.0 && !motion.fault() && motion.reached(),
      "Erhoehte Laufreibung erzeugt keinen dauerhaften zu schwachen PWM-37-Betrieb");
  }

  testLoadedUpwardCruiseConsistency();
  testMotionTraceReadOnly();
  testLoadedUpwardNoisySpeed();
  testLoadedDownwardSpeed();
  testLoadedUpwardRelease();
  testLoadedUpwardTorqueReserve();
  testCameraRecoveryOverspeed();
  testLoadedDownwardCoast();
  testCameraRunningLoad();
  testStartupWithIncreasingRunningFriction();
  testEarlyReliefCannotExtendBlockedStart();
  testStartupVelocityPeak();
  testMeasuredCruiseSpeed();
  testFullFrictionTravel();
  testTravelRangeAndStop();
  testTravelFaultsAndReplan();
  testNoisyTiltStall();
  testContinuousFrictionTilt();
  testBenchCruiseStages();
  testBoundedBenchDrive();
  testTrackingPauseDirection();
  testStaticFrictionDrive();
  testPanRunningLoadRecovery();
  testPanNearTargetNoise();
  testPanFinalBreakaway();
  {
    MotionController braking; initialize(braking); braking.command(10,0,200,200);
    bool sawBrake=false, noFalseReversal=true;
    for (uint32_t now=200; now<=20000 && !braking.fault(); now+=20)
    {
      AxisMotionStatus before=braking.status(false);
      double position=fmin(9.80,fmax(0.0,before.reference-(now<4000 ? 0.10 : 0.01)));
      braking.tick(now,position,0,true,true);
      AxisMotionStatus after=braking.status(false);
      if(now>=4000 && after.referenceVelocity<after.velocity && after.target>after.position)
      { sawBrake=true; noFalseReversal=noFalseReversal && after.phase!=MOTION_REVERSING; }
    }
    check(sawBrake && noFalseReversal && !braking.fault(),
      "Bremsen bei weiter vorausliegendem Ziel erzeugt weder fiktive Umkehr noch Umkehrtimeout");
  }
  testManualStartWithEncoderNoise();
  testProfileBoundsAndContinuousReplan(); testArrivalHoldingAndReplan();
  testFaultsAndBacklashBudget(); testSoftLoadSimulation(); testSlowProfilesAndClockWrap(); testNonblockingTransport();
  return motionChecks-initialMotionCheckCount;
}









