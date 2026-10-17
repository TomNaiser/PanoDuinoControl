#include <cstdlib>
#include <iostream>
#include <deque>
#include <string>
#include "PanoramaExecutor.h"
#include "ProtocolV2.h"
#include "V2Controller.h"
#include "TargetArrivalMonitor.h"
#include "Wire.h"
#include "VersionInfo.h"

static int tests = 0;
static std::string frame(uint16_t sequence, const std::string &type, const std::string &fields);

static void assertTrue(bool condition, const char *message)
{
  tests++;
  if (!condition)
  {
    std::cerr << "FEHLER: " << message << std::endl;
    std::exit(1);
  }
}

class FakeHardware : public IPanoramaHardware
{
public:
  FakeHardware() : reached(false), camera(false), stopped(false), pan(0), tilt(0), targetCalls(0), progressRequired(false), tiltZero(0), panTempo(0), tiltTempo(0) {}

  bool setTargetMdeg(int32_t panMdeg, int32_t tiltMdeg, uint16_t panSpeed, uint16_t tiltSpeed, bool requireMeasuredProgress)
  {
    pan = panMdeg;
    tilt = tiltMdeg + tiltZero;
    reached = false;
    stopped = false;
    targetCalls++;
    progressRequired = requireMeasuredProgress;
    panTempo = panSpeed; tiltTempo = tiltSpeed;
    return true;
  }
  bool targetsReached() { return reached; }
  bool stoppedPositionStable(int32_t,int32_t) const { return reached; }
  void setCameraActive(bool active) { camera = active; }
  void stopAllMotion() { stopped = true; }
  int32_t actualPanMdeg() const { return pan; }
  int32_t actualTiltMdeg() const { return tilt - tiltZero; }
  bool setTiltZeroOffsetMdeg(int32_t value) { tiltZero=value; stopped=true; return true; }
  void getMotionDiagnostics(MotionDiagnosticsV2 &diagnostics) const
  {
    diagnostics.sampleSequence = 0;
    diagnostics.stableSamples = reached ? 5 : 0;
    diagnostics.requiredStableSamples = 5;
    diagnostics.panWithinTolerance = reached;
    diagnostics.tiltWithinTolerance = reached;
    diagnostics.panTargetMdeg = pan;
    diagnostics.tiltTargetMdeg = tilt - tiltZero;
    diagnostics.panErrorMdeg = 0;
    diagnostics.tiltErrorMdeg = 0;
  }

  bool reached;
  bool camera;
  bool stopped;
  int32_t pan;
  int32_t tilt;
  int targetCalls;
  bool progressRequired;
  int32_t tiltZero;
  uint16_t panTempo, tiltTempo;
};

class FakeStream : public Stream
{
public:
  int available() { return static_cast<int>(input.size()); }
  int peek() { return input.empty() ? -1 : static_cast<unsigned char>(input.front()); }
  int read()
  {
    if (input.empty()) return -1;
    int value = static_cast<unsigned char>(input.front());
    input.pop_front();
    return value;
  }
  std::size_t print(const char *value)
  {
    output += value;
    return std::char_traits<char>::length(value);
  }
  std::size_t write(unsigned char value)
  {
    output += static_cast<char>(value);
    return 1;
  }
  void feed(const std::string &value)
  {
    for (std::string::const_iterator it = value.begin(); it != value.end(); ++it) input.push_back(*it);
  }

  std::deque<char> input;
  std::string output;
};

class FakeEvents : public IExecutorEvents
{
public:
  FakeEvents() : images(0), stateChanges(0), lastCaptureSequence(0), lastRevision(0) {}
  void onStateChanged(RunStateV2) { stateChanges++; }
  void onPhaseChanged(RunPhaseV2, RunPhaseV2) {}
  void onImageCompleted(const TargetPositionV2 &, int32_t, int32_t, uint32_t captureSequence, uint16_t revision)
  {
    images++;
    lastCaptureSequence = captureSequence;
    lastRevision = revision;
  }
  int images;
  int stateChanges;
  uint32_t lastCaptureSequence;
  uint16_t lastRevision;
};

static PanoramaPlanV2 makePlan(uint32_t runId, uint16_t columns)
{
  PanoramaPlanV2 plan;
  plan.runId = runId;
  plan.columns = columns;
  plan.rows = 1;
  plan.startPanMdeg = 0;
  plan.startTiltMdeg = 0;
  plan.stepPanMdeg = 1000;
  plan.stepTiltMdeg = 0;
  plan.settleTimeMs = 10;
  plan.triggerPulseMs = 30;
  plan.exposureWaitMs = 50;
  plan.panSpeedPermille = 500;
  plan.tiltSpeedPermille = 500;
  return plan;
}

static void completeMovementAndBeginExposure(PanoramaExecutor &executor, FakeHardware &hardware, uint32_t base)
{
  executor.tick(base);       // Ziel wählen
  hardware.reached = true;
  executor.tick(base + 1);   // MOVING -> SETTLING
  executor.tick(base + 11);  // SETTLING -> TRIGGER_ACTIVE
  assertTrue(hardware.camera, "Kameraausgang muss während des Triggerpulses aktiv sein");
}

static void completeOneImage(PanoramaExecutor &executor, FakeHardware &hardware, uint32_t base)
{
  completeMovementAndBeginExposure(executor, hardware, base);
  executor.tick(base + 41);
  executor.tick(base + 91);
  executor.tick(base + 92);
}

class ReturnTempoHardware : public FakeHardware
{
public:
  ReturnTempoHardware():returnTempo(800),returnCalls(0),stepBudgetMs(3000),resting(true),captureStable(true) {}
  bool resting, captureStable;
  bool stoppedPositionResting() const { return resting; }
  bool stoppedPositionStable(int32_t, int32_t) const { return resting && captureStable; }
  uint16_t columnReturnTiltTempo(uint16_t rowTempo) const
  { ++returnCalls; return returnTempo ? returnTempo : rowTempo; }
  uint16_t returnTempo;
  mutable unsigned int returnCalls;
  uint32_t stepBudgetMs;
  uint32_t incrementalMoveMaximumDurationMs() const { return stepBudgetMs; }
};

static void testIncrementalMoveDeadline()
{
  for(unsigned int variant=0;variant<4;++variant)
  {
    ReturnTempoHardware hardware; FakeEvents events; SafetyController safety;
    PanoramaExecutor executor(hardware,safety,events);
    PanoramaPlanV2 plan=makePlan(600+variant,2); plan.rows=2; plan.stepTiltMdeg=1000;
    if(variant==1) hardware.stepBudgetMs=700;
    if(variant==3) hardware.stepBudgetMs=120000;
    assertTrue(std::string(executor.loadPlan(plan))=="PLAN_ACCEPTED" &&
      std::string(executor.start(plan.runId,0))=="STARTED","Zeitbegrenzter Zeilenplan startet");
    completeOneImage(executor,hardware,0);
    uint32_t start=variant==2 ? UINT32_MAX-100 : 100;
    executor.tick(start);
    executor.tick(start+hardware.stepBudgetMs-1);
    assertTrue(executor.state()==RUN_RUNNING && !hardware.stopped,"Vor Ablauf des individuellen Budgets kein vorzeitiger Stop");
    executor.tick(start+hardware.stepBudgetMs);
    assertTrue(executor.state()==RUN_RUNNING && executor.phase()==PHASE_SETTLING && hardware.stopped && !hardware.camera && !safety.motionEnabled() &&
      std::string(executor.lastError())=="STEP_MOVE_TIMEOUT","An der Grenze Motoren aus, anschliessend passive Beruhigung auch beim Uhrueberlauf");
    int calls=hardware.targetCalls;
    hardware.resting=false; executor.tick(start+hardware.stepBudgetMs+100);
    assertTrue(!hardware.camera && hardware.targetCalls==calls,"Unruhiger Aufbau nach Timeout loest weder Foto noch Zielkorrektur aus");
    hardware.resting=true; executor.tick(start+hardware.stepBudgetMs+109);
    assertTrue(!hardware.camera,"Volle Beruhigungszeit seit letzter Unruhe wird eingehalten");
    executor.tick(start+hardware.stepBudgetMs+110);
    assertTrue(hardware.camera && !hardware.reached && !safety.motionEnabled(),"Foto am erreichten Ort ohne Behauptung, das Planziel sei erreicht");
    executor.tick(start+hardware.stepBudgetMs+140);
    executor.tick(start+hardware.stepBudgetMs+190);
    executor.tick(start+hardware.stepBudgetMs+191);
    assertTrue(executor.completedImages()==2 && executor.nextImageIndex()==2 && hardware.targetCalls==calls,
      "Timeout-Foto wird einmal verbucht; das verfehlte Ziel wird nicht erneut gefahren");
    executor.tick(start+hardware.stepBudgetMs+192);
    assertTrue(hardware.targetCalls==calls+1 && executor.currentTarget().imageIndex==2,
      "Nach dem Foto folgt die naechste geplante Position, nicht ein Schritt ab der verfehlten Istposition");
  }
  {
    ReturnTempoHardware hardware; FakeEvents events; SafetyController safety;
    PanoramaExecutor executor(hardware,safety,events);
    PanoramaPlanV2 plan=makePlan(610,2); plan.rows=2; plan.stepTiltMdeg=1000;
    plan.settleTimeMs=5000;
    assertTrue(std::string(executor.loadPlan(plan))=="PLAN_ACCEPTED" && std::string(executor.start(610,0))=="STARTED",
      "Separat verlangte Beruhigungszeit bleibt einstellbar");
    executor.tick(0); hardware.reached=true; executor.tick(1); executor.tick(5001); executor.tick(5031);
    executor.tick(5081); executor.tick(5082); executor.tick(5100);
    hardware.reached=true; executor.tick(5101); executor.tick(8100);
    assertTrue(executor.state()==RUN_RUNNING && executor.phase()==PHASE_SETTLING,
      "Schrittgrenze verlaengert die Fahrt nicht, kuerzt aber keine nach Zielruhe verlangte Beruhigungszeit");
  }
  {
    ReturnTempoHardware hardware; FakeEvents events; SafetyController safety;
    PanoramaExecutor executor(hardware,safety,events);
    PanoramaPlanV2 plan=makePlan(611,2); plan.rows=2; plan.stepTiltMdeg=1000;
    executor.loadPlan(plan); executor.start(611,0);
    completeOneImage(executor,hardware,0); completeOneImage(executor,hardware,100);
    executor.tick(200); executor.tick(3200);
    assertTrue(executor.state()==RUN_RUNNING && !hardware.stopped,"Lange Spaltenrueckfahrt bekommt nicht das kurze Zeilenschrittbudget");
  }
  {
    ReturnTempoHardware hardware; FakeEvents events; SafetyController safety;
    PanoramaExecutor executor(hardware,safety,events);
    PanoramaPlanV2 plan=makePlan(612,2); plan.rows=2; plan.stepTiltMdeg=1000;
    executor.loadPlan(plan); executor.start(612,0); completeOneImage(executor,hardware,0);
    executor.tick(100); executor.tick(3100); executor.tick(3110);
    hardware.captureStable=false; executor.tick(3111);
    assertTrue(executor.state()==RUN_FAULT && !hardware.camera && executor.completedImages()==1 &&
      std::string(executor.lastError())=="EXPOSURE_UNSTABLE","Drift waehrend Timeout-Foto stoppt den Auftrag ohne falsche Bildverbuchung");
  }
  {
    ReturnTempoHardware hardware; FakeStream stream; V2Controller controller(stream,hardware);
    controller.begin(0); stream.feed(frame(650,"HELLO","HOST_TEST")); controller.tick(0);
    stream.feed(frame(651,"MANUAL_MOVE","0|2000|500|500|1")); controller.tick(10);
    controller.tick(3009);
    assertTrue(!hardware.stopped,"Diagnostischer inkrementeller Fahrbefehl laeuft bis zu seiner Grenze");
    controller.tick(3010);
    assertTrue(hardware.stopped && !hardware.camera && stream.output.find("|STATUS|0|STOPPED|NONE|")!=std::string::npos &&
      stream.output.find("|STEP_MOVE_TIMEOUT|")!=std::string::npos,"Standalone-Diagnoseschritt stoppt firmwareseitig nach 3000 ms");
    int calls=hardware.targetCalls; stream.feed(frame(651,"MANUAL_MOVE","0|2000|500|500|1")); controller.tick(4000);
    assertTrue(hardware.targetCalls==calls && hardware.stopped,"Spaeter Retry des abgelaufenen Befehls startet keine zweite Fahrt");
  }
}

class PassiveExposureHardware : public FakeHardware
{
public:
  bool stable=true;
  bool exposurePositionStable(int32_t,int32_t) { return stable; }
  bool stoppedPositionStable(int32_t,int32_t) const { return stable; }
  void stopAllMotion() { FakeHardware::stopAllMotion(); reached=false; }
};

static void testPassiveExposureDoesNotReconfirmArrival()
{
  PassiveExposureHardware hardware; FakeEvents events; SafetyController safety;
  PanoramaExecutor executor(hardware,safety,events);
  PanoramaPlanV2 plan=makePlan(614,1); executor.loadPlan(plan); executor.start(614,0);
  completeMovementAndBeginExposure(executor,hardware,0);
  assertTrue(!hardware.reached && hardware.stopped && !safety.motionEnabled(),
    "Vor Aufnahme Motoren aus; Ruecksetzen der Ankunftsbestaetigung ist kein Belichtungsfehler");
  executor.tick(41); executor.tick(91); executor.tick(92);
  assertTrue(executor.state()==RUN_COMPLETED && executor.completedImages()==1,
    "Bestaetigte Aufnahme nutzt passive Stabilitaet statt erneutem strengen Ankunftszaehler");
  executor.clearRun(); assertTrue(executor.plan().runId==0,"Entfernter Testauftrag gibt Konfiguration wieder frei");

  FakeEvents driftEvents; SafetyController driftSafety;
  PanoramaExecutor drift(hardware,driftSafety,driftEvents);
  drift.loadPlan(plan); drift.start(614,100); completeMovementAndBeginExposure(drift,hardware,100);
  hardware.stable=false; drift.tick(112);
  assertTrue(drift.state()==RUN_FAULT && !hardware.camera && driftEvents.images==0,
    "Echte passive Instabilitaet bleibt ein Belichtungsfehler mit sofortiger Abschaltung");
}

static void testSeparateColumnReturnTempo()
{
  for(unsigned int variant=0;variant<4;++variant)
  {
    ReturnTempoHardware hardware; FakeEvents events; SafetyController safety;
    PanoramaExecutor executor(hardware,safety,events);
    PanoramaPlanV2 plan=makePlan(500+variant,2);
    if(variant==2) { plan.rows=1; }
    else { plan.rows=3; plan.startTiltMdeg=2000; plan.stepTiltMdeg=-2000; }
    if(variant==1) hardware.returnTempo=0;
    if(variant==3)
    {
      plan.moveMode=1; plan.planVersion=2; plan.rows=2; plan.columns=3;
      plan.startTiltMdeg=0; plan.endTiltMdeg=-2000; plan.endPanMdeg=2000;
    }
    assertTrue(std::string(executor.loadPlan(plan))=="PLAN_ACCEPTED" &&
      std::string(executor.start(plan.runId,0))=="STARTED","Tempovergleichsplan angenommen und gestartet");
    for(uint32_t i=0;i<plan.totalImages();++i)
    {
      uint32_t base=i*100; executor.tick(base);
      uint16_t expected=variant==0 && i==3 ? 800 : 500;
      assertTrue(hardware.panTempo==500 && hardware.tiltTempo==expected,
        "Nur spaetere Raster-Spaltenrueckfahrt schneller; Pan, Zeilen, erstes Ziel und sphaerische Baender unveraendert");
      completeOneImage(executor,hardware,base);
    }
    assertTrue(hardware.returnCalls==(variant<2 ? 1U : 0U),
      "Rueckfahrtempo nicht bei einzeiligem Raster oder sphaerischen Rezepten abfragen");
  }
}

static void testPauseCompletesCurrentImage()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(100, 2);
  assertTrue(std::string(executor.loadPlan(plan)) == "PLAN_ACCEPTED", "Plan annehmen");
  assertTrue(std::string(executor.start(100, 0)) == "STARTED", "Plan starten");
  executor.tick(0);
  assertTrue(!hardware.progressRequired, "Rasterziel verwendet historische Zielerkennung ohne manuelle Fortschrittspruefung");
  completeMovementAndBeginExposure(executor, hardware, 0);
  assertTrue(std::string(executor.pause(100)) == "PAUSE_PENDING", "Pause während Bild muss ausstehend sein");
  executor.tick(41);  // Trigger beenden
  assertTrue(!hardware.camera, "Trigger muss nach Pulsende aus sein");
  executor.tick(91);  // Belichtungswartezeit beenden
  executor.tick(92);  // Bild verbuchen und pausieren
  assertTrue(executor.state() == RUN_PAUSED, "Pause erst nach vollständig verbuchtem Bild");
  assertTrue(executor.completedImages() == 1 && events.images == 1, "Aktuelles Bild genau einmal verbuchen");
  assertTrue(hardware.stopped && !safety.motionEnabled(), "Motoren in Pause abschalten");
}

static void testStopIsImmediate()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(200, 1);
  executor.loadPlan(plan);
  executor.start(200, 0);
  executor.tick(0);
  assertTrue(std::string(executor.stop(200, "USER_STOP")) == "STOPPED", "Stop annehmen");
  assertTrue(executor.state() == RUN_STOPPED, "Stopzustand setzen");
  assertTrue(hardware.stopped && !hardware.camera && !safety.motionEnabled(), "Stop schaltet Motor und Kamera sofort ab");
  executor.tick(10000);
  assertTrue(events.images == 0, "Nach Stop kein Bild mehr erzeugen");
}

static void testMovementTimeoutStopsMotion()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(250, 1);
  executor.loadPlan(plan);
  executor.start(250, 0);
  executor.tick(0);
  executor.tick(120000);
  assertTrue(executor.state() == RUN_FAULT, "Bewegungs-Timeout erzeugt Fehlerzustand");
  assertTrue(hardware.stopped && !safety.motionEnabled(), "Bewegungs-Timeout schaltet Motoren ab");
  assertTrue(std::string(executor.lastError()) == "MOVEMENT_TIMEOUT", "Bewegungs-Timeout im Status erhalten");
}

static void testTargetArrivalRequiresFreshStableSamples()
{
  TargetArrivalMonitor monitor(5);
  monitor.resetForNewTarget();
  assertTrue(!monitor.reached(), "Neues Ziel darf ohne frischen Encoderwert nicht erreicht sein");
  for (int sample = 0; sample < 4; sample++) monitor.observe(true, true);
  assertTrue(!monitor.reached(), "Weniger als fuenf stabile Samples duerfen nicht ausreichen");
  monitor.observe(true, true);
  assertTrue(monitor.reached(), "Fuenf stabile Encoder-Samples bestaetigen das Ziel");
  monitor.observe(true, false);
  assertTrue(!monitor.reached() && monitor.stableSamples() == 0, "Ein instabiles Sample verwirft die Zielbestaetigung");
}

static void testSmallManualTargetNeedsMeasuredProgress()
{
  TargetArrivalMonitor monitor(5);
  monitor.resetForNewTarget(true, false);
  // A one-degree target is initially inside the old 1.5-degree tolerance.
  // Five unchanged encoder readings must not stop the motor before it moves.
  for (int sample = 0; sample < 8; sample++) monitor.observe(true, true, false, true);
  assertTrue(!monitor.reached(), "Unbewegter Motor darf kleines Ziel nicht vorzeitig erreichen");
  for (int sample = 0; sample < 5; sample++) monitor.observe(true, true, true, true);
  assertTrue(monitor.reached(), "Nach messbarer Bewegung und stabilen Samples Ziel erreicht");
}

static void testDebugEventRingIsBounded()
{
  assertTrue(sizeof(DebugEventRecordV2) <= 32, "Ein Debug-Ereignis bleibt fuer Nano-SRAM kompakt");
  assertTrue(sizeof(DebugEventRecordV2) * DEBUG_EVENT_CAPACITY_V2 <= 256, "Gesamter Debug-Ring belegt hoechstens 256 Bytes");
  DebugEventLogV2 log;
  DebugEventRecordV2 record = {};
  record.code = DEBUG_TARGET_SELECTED;
  for (uint8_t index = 0; index < 10; index++)
  {
    record.deviceTimeMs = index;
    log.append(record);
  }
  assertTrue(log.count() == DEBUG_EVENT_CAPACITY_V2, "Debug-Ring bleibt auf Nano-feste Kapazitaet begrenzt");
  assertTrue(log.dropped() == (uint16_t)(10 - DEBUG_EVENT_CAPACITY_V2), "Ueberschriebene Debug-Ereignisse zaehlen");
  assertTrue(log.record(0).sequence == (uint16_t)(11 - DEBUG_EVENT_CAPACITY_V2) && log.newestSequence() == 10,
             "Debug-Ring behaelt die neuesten Ereignisse geordnet");
}

static void testSettlingRevalidatesTargetAndKeepsMovementTimeout()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(260, 1);
  plan.settleTimeMs = 100;
  executor.loadPlan(plan);
  executor.start(260, 0);
  executor.tick(0);
  hardware.reached = true;
  executor.tick(119000);
  assertTrue(executor.phase() == PHASE_SETTLING && !hardware.camera, "Zielerreichung startet nur die Beruhigungsphase");
  hardware.reached = false;
  executor.tick(119100);
  assertTrue(executor.phase() == PHASE_SETTLING && !hardware.camera && hardware.stopped && !safety.motionEnabled(), "Drift nach bestaetigtem Halt wartet stromlos auf passive Ruhe");
  executor.tick(120000);
  assertTrue(executor.state() == RUN_FAULT, "Passive Beruhigung darf die Abbruchgrenze nicht neu starten");
  assertTrue(events.images == 0 && !hardware.camera, "Ohne dauerhaft erreichtes Ziel keine Aufnahme");
}

static void testRewindCreatesRevisionWithoutDecrementingCaptureCount()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(275, 5);
  executor.loadPlan(plan);
  executor.start(275, 0);
  completeOneImage(executor, hardware, 0);
  completeOneImage(executor, hardware, 100);
  completeOneImage(executor, hardware, 200);
  completeOneImage(executor, hardware, 300);
  assertTrue(executor.completedImages() == 4 && executor.nextImageIndex() == 4, "Vier reguläre Bilder vor Rücksprung");
  assertTrue(std::string(executor.pause(275)) == "PAUSED", "Zwischen Bildern sofort pausieren");
  assertTrue(std::string(executor.rewindColumns(275, 3)) == "REWOUND", "Drei Spalten zurücksetzen");
  assertTrue(executor.nextImageIndex() == 1 && executor.planRevision() == 1, "Fortsetzungspunkt und Revision setzen");
  assertTrue(executor.segmentCount() == 2 && executor.segment(0).captureCount == 4 && executor.segment(1).startImageIndex == 1,
             "Kompakte Segmente für Reconnect speichern");
  executor.resume(275);
  completeOneImage(executor, hardware, 400);
  assertTrue(executor.completedImages() == 5, "Aufnahmezähler trotz wiederholtem Rasterindex monoton halten");
  assertTrue(events.lastCaptureSequence == 5 && events.lastRevision == 1, "Wiederholtes Bild eindeutig der Revision zuordnen");
}

static void testRunningRewindWaitsForCurrentImageAndCombinesClicks()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(276, 5);
  plan.rows = 2;
  assertTrue(std::string(executor.loadPlan(plan)) == "PLAN_ACCEPTED", "Mehrzeiliges Raster annehmen");
  executor.start(276, 0);
  for (uint32_t base = 0; base <= 400; base += 100) completeOneImage(executor, hardware, base);
  assertTrue(executor.nextImageIndex() == 5, "Test steht in zweiter Zeile der dritten Spalte");
  completeMovementAndBeginExposure(executor, hardware, 500);
  assertTrue(std::string(executor.rewindColumns(276, 1)) == "REWIND_PENDING", "Ruecksprung waehrend Trigger vormerken");
  assertTrue(std::string(executor.rewindColumns(276, 1)) == "REWIND_PENDING", "Zwei Klicks in einem Ruecksprung sammeln");
  assertTrue(executor.nextImageIndex() == 5 && executor.planRevision() == 0 && hardware.camera,
             "Aktuelles Bild bleibt bis zum Commit unveraendert");
  executor.tick(541);
  executor.tick(591);
  executor.tick(592);
  assertTrue(executor.completedImages() == 6 && events.lastRevision == 0, "Aktuelles Bild genau einmal mit alter Revision verbuchen");
  assertTrue(executor.nextImageIndex() == 2 && executor.planRevision() == 1 && executor.segmentCount() == 2,
             "Zwei Spalten zurueck zu Zeile null mit nur einem neuen Segment");
  assertTrue(executor.state() == RUN_RUNNING && executor.pendingRewindColumns() == 0,
             "Raster setzt ohne manuelle Pause fort");
  executor.tick(600);
  assertTrue(executor.currentTarget().column == 1 && executor.currentTarget().row == 0,
             "Erstes Wiederholungsbild liegt in der frueheren Spalte");
}

static void testRunningRewindProtocolAcknowledgesAndPublishesSegment()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(60, "HELLO", "REWIND_TEST"));
  controller.tick(1);
  stream.feed(frame(61, "LOAD_PLAN", "900|1|0|0|0|3|1|1000|0|10|30|50|500|500|0|0"));
  controller.tick(2);
  stream.feed(frame(62, "START", "900"));
  controller.tick(3);
  hardware.reached = true;
  controller.tick(4);
  controller.tick(14);
  stream.output.clear();
  stream.feed(frame(63, "REWIND_COLUMNS", "900|1"));
  controller.tick(15);
  assertTrue(stream.output.find("|ACK|63|REWIND_PENDING|") != std::string::npos,
             "Laufender Ruecksprung wird als vorgemerkt bestaetigt");
  controller.tick(44);
  controller.tick(94);
  controller.tick(95);
  assertTrue(stream.output.find("|RUN_SEGMENT|900|1|0|0|") != std::string::npos,
             "Neues Segment wird nach dem laufenden Bild an Client gemeldet");
  assertTrue(stream.output.find("|IMAGE_COMPLETED|") != std::string::npos,
             "Laufendes Bild bleibt beim Ruecksprung erhalten");
}

static void testForwardColumnsSkipsRemainderAndKeepsCaptureCount()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(278, 5);
  plan.rows = 2;
  assertTrue(std::string(executor.loadPlan(plan)) == "PLAN_ACCEPTED", "Mehrzeiliges Vorwaertsraster annehmen");
  executor.start(278, 0);
  completeOneImage(executor, hardware, 0);
  completeOneImage(executor, hardware, 100);
  assertTrue(executor.nextImageIndex() == 2, "Vorwaertstest steht in erster Zeile der zweiten Spalte");
  completeMovementAndBeginExposure(executor, hardware, 200);
  assertTrue(std::string(executor.forwardColumns(278, 1)) == "FORWARD_PENDING", "Vorwaertssprung waehrend Trigger vormerken");
  assertTrue(std::string(executor.rewindColumns(278, 1)) == "NAVIGATION_PENDING", "Gegenrichtung vor Bildabschluss ablehnen");
  assertTrue(executor.nextImageIndex() == 2 && hardware.camera, "Aktuelles Bild bleibt unveraendert");
  executor.tick(241);
  executor.tick(291);
  executor.tick(292);
  assertTrue(executor.completedImages() == 3 && events.lastRevision == 0, "Laufendes Bild einmalig verbuchen");
  assertTrue(executor.nextImageIndex() == 4 && executor.planRevision() == 1 && executor.segmentCount() == 2,
             "Rest der aktuellen Spalte ueberspringen und Segment anlegen");
  assertTrue(executor.segment(1).startImageIndex == 4 && executor.segment(1).captureCount == 0 &&
             executor.lastNavigationForward(), "Vorwaertsziel fuer Reconnect protokollieren");
  executor.tick(300);
  assertTrue(executor.currentTarget().column == 2 && executor.currentTarget().row == 0,
             "Naechstes Bild ist erste Zeile der Folgespalte");
}

static void testPausedForwardCombinesClicksAndStopsAtLastColumn()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(279, 4);
  plan.rows = 2;
  executor.loadPlan(plan);
  executor.start(279, 0);
  assertTrue(std::string(executor.pause(279)) == "PAUSED", "Vor erster Aufnahme pausieren");
  assertTrue(std::string(executor.forwardColumns(279, 2)) == "FORWARDED", "Zwei Spalten sofort vorwaerts");
  assertTrue(executor.nextImageIndex() == 4 && executor.segment(1).startImageIndex == 4 &&
             executor.completedImages() == 0, "Pause setzt nur Rastercursor, nicht Bildzaehler");
  assertTrue(std::string(executor.forwardColumns(279, 2)) == "LAST_COLUMN", "Sprung ueber letzte Spalte ablehnen");
  assertTrue(std::string(executor.forwardColumns(279, 1)) == "FORWARDED", "Bis zur letzten Spalte vorwaerts");
  assertTrue(executor.nextImageIndex() == 6 && executor.planRevision() == 2, "Letzte Spalte korrekt ansteuern");
  assertTrue(std::string(executor.forwardColumns(279, 1)) == "LAST_COLUMN", "Am Rasterende nicht vorwaertsspringen");
}

static void testRunningForwardProtocolPublishesSegment()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(70, "HELLO", "FORWARD_TEST"));
  controller.tick(1);
  stream.feed(frame(71, "LOAD_PLAN", "901|1|0|0|0|3|2|1000|0|10|30|50|500|500|0|0"));
  controller.tick(2);
  stream.feed(frame(72, "START", "901"));
  controller.tick(3);
  hardware.reached = true;
  controller.tick(4);
  controller.tick(14);
  stream.output.clear();
  stream.feed(frame(73, "FORWARD_COLUMNS", "901|1"));
  controller.tick(15);
  assertTrue(stream.output.find("|ACK|73|FORWARD_PENDING|") != std::string::npos,
             "Laufender Vorwaertssprung wird vorgemerkt");
  controller.tick(44);
  controller.tick(94);
  controller.tick(95);
  assertTrue(stream.output.find("|IMAGE_COMPLETED|") != std::string::npos,
             "Aktuelles Bild beim Vorwaertssprung erhalten");
  assertTrue(stream.output.find("|RUN_SEGMENT|901|1|2|0|") != std::string::npos,
             "Vorwaertssegment an Client melden");
}

static void testTimingUpdateStartsWithNextImage()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(277, 2);
  assertTrue(std::string(executor.loadPlan(plan)) == "PLAN_ACCEPTED", "Plan fuer Laufzeit-Zeiten annehmen");
  executor.start(277, 0);
  completeMovementAndBeginExposure(executor, hardware, 0);
  assertTrue(std::string(executor.updateTiming(999, 200, 100, 300)) == "RUN_ID_MISMATCH",
             "Zeitupdate fuer fremden Lauf ablehnen");
  assertTrue(std::string(executor.updateTiming(277, 200, 0, 300)) == "VALUE_OUT_OF_RANGE",
             "Ungueltigen Triggerpuls ablehnen");
  assertTrue(std::string(executor.updateTiming(277, 200, 100, 300)) == "TIMING_QUEUED",
             "Zeitupdate waehrend Ausloesung vormerken");
  assertTrue(executor.plan().triggerPulseMs == 30 && hardware.camera,
             "Laufenden Triggerpuls nicht veraendern");
  executor.tick(41);
  assertTrue(!hardware.camera && executor.phase() == PHASE_EXPOSURE_WAIT,
             "Aktuellen Puls mit bisheriger Dauer beenden");
  executor.tick(91);
  executor.tick(92);
  assertTrue(executor.completedImages() == 1 && executor.plan().settleTimeMs == 200 &&
             executor.plan().triggerPulseMs == 100 && executor.plan().exposureWaitMs == 300 &&
             executor.timingRevision() == 1,
             "Alle drei neuen Zeiten gelten ab dem naechsten Bild");
  executor.tick(100);
  hardware.reached = true;
  executor.tick(101);
  executor.tick(300);
  assertTrue(!hardware.camera, "Neue Beruhigungszeit vor dem naechsten Bild einhalten");
  executor.tick(301);
  assertTrue(hardware.camera, "Naechsten Trigger nach neuer Beruhigungszeit starten");
  executor.tick(400);
  assertTrue(hardware.camera, "Neuen Triggerpuls nicht vorzeitig beenden");
  executor.tick(401);
  assertTrue(!hardware.camera, "Neuen Triggerpuls nach 100 ms beenden");
  executor.tick(700);
  assertTrue(executor.phase() == PHASE_EXPOSURE_WAIT, "Neue Belichtungswartezeit einhalten");
  executor.tick(701);
  executor.tick(702);
  assertTrue(executor.completedImages() == 2 && executor.state() == RUN_COMPLETED,
             "Lauf mit aktualisierten Zeiten abschliessen");
}

static void testTimingUpdateProtocolPublishesAppliedPlan()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(70, "HELLO", "TIMING_TEST"));
  controller.tick(1);
  stream.feed(frame(71, "LOAD_PLAN", "910|1|0|0|0|2|1|1000|0|10|30|50|500|500|0|0"));
  controller.tick(2);
  stream.feed(frame(72, "START", "910"));
  controller.tick(3);
  hardware.reached = true;
  controller.tick(4);
  controller.tick(14);
  stream.output.clear();
  stream.feed(frame(73, "UPDATE_TIMING", "910|200|100|300"));
  controller.tick(15);
  assertTrue(stream.output.find("|ACK|73|TIMING_QUEUED|") != std::string::npos,
             "Laufendes Bild bestaetigt vorgemerkte Zeiten");
  assertTrue(stream.output.find("|PLAN|") == std::string::npos,
             "Noch nicht wirksame Zeiten nicht als aktiven Plan melden");
  controller.tick(44);
  controller.tick(94);
  controller.tick(95);
  assertTrue(stream.output.find("|200|100|300|") != std::string::npos &&
             stream.output.find("|PLAN|") != std::string::npos,
             "Firmware sendet neuen aktiven Plan nach Bildabschluss");
}

static void testTiltLimitRejectsWholePlan()
{
  FakeHardware hardware;
  FakeEvents events;
  SafetyController safety;
  PanoramaExecutor executor(hardware, safety, events);
  PanoramaPlanV2 plan = makePlan(300, 1);
  plan.startTiltMdeg = TILT_HARD_MAX_MDEG + 1;
  assertTrue(std::string(executor.loadPlan(plan)) == "TILT_LIMIT", "Tilt-Grenzüberschreitung vor Start ablehnen");
  assertTrue(executor.state() == RUN_IDLE && hardware.targetCalls == 0, "Unsicherer Plan darf keine Fahrt auslösen");
}

static void testNanoPlanImageLimit()
{
  SafetyController safety;
  PanoramaPlanV2 maximum = makePlan(301, 40);
  maximum.rows = 100;
  assertTrue(std::string(maximum.validate(safety)) == "OK", "Nano-Plan mit genau 4000 Bildern akzeptieren");

  PanoramaPlanV2 tooLarge = makePlan(302, 4001);
  tooLarge.stepPanMdeg = 0;
  assertTrue(std::string(tooLarge.validate(safety)) == "VALUE_OUT_OF_RANGE", "Nano-Plan ueber 4000 Bilder ablehnen");
}

static std::string frame(uint16_t sequence, const std::string &type, const std::string &fields)
{
  std::string body = "@P2|" + std::to_string(sequence) + "|" + type;
  if (!fields.empty()) body += "|" + fields;
  char suffix[10];
  std::snprintf(suffix, sizeof(suffix), "|%04X\r\n", ProtocolV2::crc16(body.c_str(), body.size()));
  return body + suffix;
}

static void testProtocolRetryIsIdempotent()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);

  stream.feed(frame(10, "HELLO", "TEST_CLIENT"));
  controller.tick(1);
  assertTrue(stream.output.find("|ACK|10|HELLO_ACK|") != std::string::npos, "HELLO bestätigen");

  stream.output.clear();
  stream.feed(frame(11, "LOAD_PLAN", "400|1|0|0|0|1|1|0|0|10|30|50|500|500|0|0"));
  controller.tick(2);
  assertTrue(stream.output.find("|ACK|11|PLAN_ACCEPTED|") != std::string::npos, "Plan über Protokoll annehmen");

  std::string start = frame(12, "START", "400");
  stream.output.clear();
  stream.feed(start);
  controller.tick(3);
  assertTrue(stream.output.find("|ACK|12|STARTED|") != std::string::npos, "START bestätigen");
  assertTrue(hardware.targetCalls == 1, "START wählt genau ein erstes Ziel");

  stream.output.clear();
  stream.feed(start);
  controller.tick(4);
  assertTrue(stream.output.find("|ACK|12|STARTED|") != std::string::npos, "Verlorenes START-ACK identisch wiederholen");
  assertTrue(hardware.targetCalls == 1, "START-Retry darf keine zweite Wirkung auslösen");

  stream.output.clear();
  stream.feed(frame(12, "START", "401"));
  controller.tick(5);
  assertTrue(stream.output.find("|NACK|12|SEQUENCE_CONFLICT|") != std::string::npos, "Gleiche Sequenz mit anderem Inhalt ablehnen");
}

static void testSafetyBoundariesAndFragmentation()
{
  assertTrue(ProtocolV2::crc16("123456789", 9) == 0x29B1, "Firmware-CRC-Referenzvektor");

  SafetyController safety;
  assertTrue(safety.validateTarget(0, TILT_HARD_MIN_MDEG) == SAFETY_OK, "Untere Tilt-Grenze einschließen");
  assertTrue(safety.validateTarget(0, TILT_HARD_MIN_MDEG - 1) == SAFETY_TILT_LIMIT, "Unterhalb der Tilt-Grenze ablehnen");
  assertTrue(safety.validateTarget(0, TILT_HARD_MAX_MDEG) == SAFETY_OK, "Obere Tilt-Grenze einschließen");
  assertTrue(safety.validateTarget(0, TILT_HARD_MAX_MDEG + 1) == SAFETY_TILT_LIMIT, "Oberhalb der Tilt-Grenze ablehnen");
  safety.enableMotion();
  assertTrue(safety.observeTilt(TILT_HARD_MAX_MDEG + 1) == SAFETY_TILT_LIMIT && !safety.motionEnabled(), "Laufzeitgrenze sperrt Bewegung");

  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  std::string hello = frame(20, "HELLO", "FRAGMENT_TEST");
  stream.feed(hello.substr(0, 7));
  controller.tick(1);
  assertTrue(stream.output.empty(), "Teilrahmen noch nicht auswerten");
  stream.feed(hello.substr(7));
  controller.tick(2);
  assertTrue(stream.output.find("|ACK|20|HELLO_ACK|") != std::string::npos, "Fragmentierten Rahmen zusammensetzen");

  FakeStream timeoutStream;
  FakeHardware timeoutHardware;
  V2Controller timeoutController(timeoutStream, timeoutHardware);
  timeoutController.begin(0);
  timeoutStream.feed(hello.substr(0, 7));
  timeoutController.tick(1);
  timeoutController.tick(400);
  timeoutStream.feed(hello.substr(7));
  timeoutController.tick(401);
  assertTrue(timeoutStream.output.empty(), "Abgelaufenen Teilrahmen ohne Seiteneffekt verwerfen");
}

static void testPausedManualCaptureAndProtocolRewind()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(30, "HELLO", "PAUSE_TEST"));
  controller.tick(1);
  assertTrue(stream.output.find("|DEVICE_INFO|" PANODUINO_FIRMWARE_VERSION "|" PANODUINO_FIRMWARE_DATE "|2|ARDUINO_NANO|") != std::string::npos,
             "Handshake meldet aktuelle Firmware- und Protokollversion");
  stream.feed(frame(29, "GET_DIAGNOSTICS", ""));
  controller.tick(1);
  assertTrue(stream.output.find("|ACK|29|OK|") != std::string::npos && stream.output.find("|DIAGNOSTICS|") != std::string::npos,
             "Read-only Diagnoseschnittstelle antwortet gerahmt");
  assertTrue(stream.output.find("|120000|0|0|") != std::string::npos,
             "Diagnoseframe enthaelt aktuellen und minimalen freien SRAM");
  stream.feed(frame(31, "LOAD_PLAN", "500|1|0|0|0|2|1|1000|0|10|30|50|500|500|0|0"));
  controller.tick(2);
  stream.feed(frame(32, "START", "500"));
  controller.tick(3);
  stream.feed(frame(33, "PAUSE", "500"));
  controller.tick(4);
  hardware.reached = true;
  controller.tick(5);
  controller.tick(15);
  controller.tick(45);
  controller.tick(95);
  controller.tick(96);
  assertTrue(stream.output.find("|PAUSED|") != std::string::npos, "Pause nach aktuellem Bild erreichen");

  stream.output.clear();
  stream.feed(frame(34, "MANUAL_MOVE", "1000|0"));
  controller.tick(97);
  assertTrue(stream.output.find("|ACK|34|MOVING|") != std::string::npos, "Manuelle Bewegung in Pause erlauben");
  assertTrue(hardware.progressRequired, "Manuelle Pfeilfahrt verlangt weiterhin gemessenen Fortschritt");
  hardware.reached = true;
  controller.tick(98);
  assertTrue(!hardware.stopped && controller.motionEnabled(), "Manuelle Zielankunft erhaelt ruhige Halteregelung");
  assertTrue(stream.output.find("|STATUS|500|PAUSED|NONE|1|2|") != std::string::npos,
             "Pfeilfahrt erhaelt pausierten Lauf und Aufnahmezaehler");

  stream.output.clear();
  stream.feed(frame(35, "TRIGGER_ONCE", "30"));
  controller.tick(99);
  controller.tick(129);
  assertTrue(stream.output.find("|MANUAL_IMAGE_TRIGGERED|") != std::string::npos, "Manuelle Aufnahme als eigenes Ereignis melden");

  stream.output.clear();
  stream.feed(frame(37, "HELLO", "RECONNECT_TEST"));
  controller.tick(129);
  stream.feed(frame(38, "GET_DEBUG_LOG", ""));
  controller.tick(130);
  assertTrue(stream.output.find("|DEBUG_EVENT|") != std::string::npos && stream.output.find("|15|") != std::string::npos,
             "Manuelle Aufnahme nach Reconnect ueber kompakten Debugring nachweisen");

  stream.output.clear();
  stream.feed(frame(36, "REWIND_COLUMNS", "500|1"));
  controller.tick(130);
  assertTrue(stream.output.find("|ACK|36|REWOUND|") != std::string::npos, "Rastercursor in Pause zurücksetzen");
  assertTrue(stream.output.find("|RUN_SEGMENT|500|1|0|0|") != std::string::npos, "Neue Rasterrevision für Reconnect senden");

  stream.output.clear();
  stream.feed(frame(40, "GET_DEBUG_LOG", "0"));
  controller.tick(131);
  assertTrue(stream.output.find("|DEBUG_EVENT|") != std::string::npos && stream.output.find("|DEBUG_LOG_END|") != std::string::npos,
             "Begrenzten Firmware-Ereignisring fuer Support ausgeben");

  stream.output.clear();
  stream.feed(frame(41, "GET_SUPPORT_BUNDLE", ""));
  controller.tick(132);
  assertTrue(stream.output.find("|DEVICE_INFO|") != std::string::npos && stream.output.find("|DIAGNOSTICS|") != std::string::npos &&
             stream.output.find("|DEBUG_LOG_END|") != std::string::npos, "Supportbundle enthaelt Version, Snapshot und Ereignisring");
}

static void testAutonomousTimelapseSurvivesReconnect()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(50, "HELLO", "PC"));
  controller.tick(1);
  stream.output.clear();
  stream.feed(frame(51, "START_TIMELAPSE", "900|100|30"));
  controller.tick(2);
  assertTrue(stream.output.find("|ACK|51|TIMELAPSE_STARTED|") != std::string::npos,
             "Autonome Timelapse bestaetigen");
  controller.tick(101);
  assertTrue(!hardware.camera, "Erste Timelapse-Aufnahme erst nach dem Intervall");
  controller.tick(102);
  assertTrue(hardware.camera, "Timelapse-Trigger ohne weitere Clientbefehle starten");
  controller.tick(132);
  assertTrue(!hardware.camera, "Timelapse-Puls selbststaendig beenden");
  assertTrue(stream.output.find("|STATUS|900|TIMELAPSE|NONE|1|0|") != std::string::npos,
             "Timelapse-Aufnahmezaehler im Status melden");
  stream.output.clear();
  stream.feed(frame(52, "HELLO", "ANDROID"));
  controller.tick(150);
  assertTrue(stream.output.find("|STATUS|900|TIMELAPSE|NONE|1|0|") != std::string::npos,
             "Neuer Client erhaelt laufenden Timelapse-Status");
  stream.output.clear();
  stream.feed(frame(53, "LOAD_PLAN", "901|1|0|0|0|1|1|0|0|10|30|50|500|500|0|0"));
  controller.tick(151);
  assertTrue(stream.output.find("|NACK|53|INVALID_STATE|") != std::string::npos,
             "Panorama waehrend Timelapse ablehnen");
  controller.tick(202);
  assertTrue(hardware.camera, "Naechste Timelapse-Aufnahme ohne PC starten");
  stream.output.clear();
  stream.feed(frame(54, "STOP_TIMELAPSE", "900"));
  controller.tick(203);
  assertTrue(!hardware.camera && stream.output.find("|ACK|54|TIMELAPSE_STOPPED|") != std::string::npos,
             "Timelapse-Stop beendet laufenden Kamerapuls sofort");
  controller.tick(402);
  assertTrue(!hardware.camera, "Nach Stop keine weitere Ausloesung");
}

static void testStoppedTimelapseCanBeCleared()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(210, "START_TIMELAPSE", "910|100|30"));
  controller.tick(1);
  stream.output.clear();
  stream.feed(frame(211, "CLEAR_RUN", ""));
  controller.tick(2);
  assertTrue(stream.output.find("|NACK|211|INVALID_STATE|") != std::string::npos,
             "Aktive Timelapse nicht durch CLEAR_RUN loeschen");
  stream.feed(frame(212, "STOP_TIMELAPSE", "910"));
  controller.tick(3);
  stream.output.clear();
  stream.feed(frame(213, "CLEAR_RUN", ""));
  controller.tick(4);
  assertTrue(stream.output.find("|ACK|213|CLEARED|") != std::string::npos,
             "Gestoppte Timelapse bei internem IDLE loeschen");
  assertTrue(stream.output.find("|STATUS|0|IDLE|NONE|0|0|") != std::string::npos && !hardware.camera,
             "CLEAR_RUN entfernt Timelapse-Zustand und meldet IDLE ohne Ausloesung");
  stream.output.clear();
  stream.feed(frame(214, "START_TIMELAPSE", "911|100|30"));
  controller.tick(5);
  assertTrue(stream.output.find("|ACK|214|TIMELAPSE_STARTED|") != std::string::npos,
             "Nach Loeschen neue Timelapse starten");
  stream.feed(frame(215, "ESTOP", "0|TEST"));
  controller.tick(6);
  stream.output.clear();
  stream.feed(frame(216, "CLEAR_RUN", ""));
  controller.tick(7);
  assertTrue(stream.output.find("|ACK|216|CLEARED|") != std::string::npos &&
             stream.output.find("|STATUS|0|IDLE|NONE|0|0|") != std::string::npos,
             "Auch nach Timelapse-Notstop beide Zustaende loeschen");
}

static void testTiltLimitsCanBeSavedReadAndRejectUnsafeChanges()
{
  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(100, "HELLO", "TILT_TEST"));
  controller.tick(1);
  stream.output.clear();
  // Firmware -50..+55 entspricht in der GUI -55..+50 Grad.
  stream.feed(frame(101, "SET_TILT_LIMITS", "-50000|55000"));
  controller.tick(2);
  assertTrue(stream.output.find("|ACK|101|TILT_LIMITS_SAVED|") != std::string::npos,
             "Gemessene Endlagen dauerhaft einstellen");
  assertTrue(stream.output.find("|CAPABILITIES|-180000|180000|-50000|55000|1|") != std::string::npos,
             "Neue Grenzen an die GUI melden");
  stream.output.clear();
  stream.feed(frame(105, "MANUAL_MOVE", "0|1000"));
  controller.tick(2);
  stream.output.clear();
  stream.feed(frame(106, "SET_TILT_LIMITS", "-45000|50000"));
  controller.tick(2);
  assertTrue(stream.output.find("|NACK|106|INVALID_STATE|") != std::string::npos,
             "Grenzen waehrend der Achsbewegung nicht aendern");
  controller.emergencyStop("TEST_STOP");
  stream.output.clear();
  stream.feed(frame(102, "SET_TILT_LIMITS", "-51000|55000"));
  controller.tick(3);
  assertTrue(stream.output.find("|NACK|102|VALUE_OUT_OF_RANGE|") != std::string::npos,
             "Werte jenseits gemessener Endlagen ablehnen");
  hardware.tilt = 54000;
  stream.output.clear();
  stream.feed(frame(103, "SET_TILT_LIMITS", "-45000|50000"));
  controller.tick(4);
  assertTrue(stream.output.find("|NACK|103|VALUE_OUT_OF_RANGE|") != std::string::npos,
             "Grenze nicht hinter aktueller Achsposition setzen");
  FakeStream rebootStream;
  FakeHardware rebootHardware;
  V2Controller rebooted(rebootStream, rebootHardware);
  rebooted.begin(0);
  rebootStream.feed(frame(104, "GET_CAPABILITIES", ""));
  rebooted.tick(1);
  assertTrue(rebootStream.output.find("|CAPABILITIES|-180000|180000|-50000|55000|1|") != std::string::npos,
             "Tilt-Grenzen nach Neustart erneut laden");
}

static void testSphericalBandsAndProtocol()
{
  SafetyController safety;
  PanoramaPlanV2 plan = makePlan(880, 5);
  plan.planVersion = 2;
  plan.moveMode = 1;
  plan.startPanMdeg = -60000;
  plan.endPanMdeg = 60000;
  plan.startTiltMdeg = -45000;
  plan.endTiltMdeg = 45000;
  plan.rows = 3;
  plan.stepPanMdeg = 30000;
  plan.stepTiltMdeg = 30000;
  assertTrue(plan.totalImages() == 13, "Spherical: 4, 5, 4 Bilder in Tilt-Baendern");
  assertTrue(std::string(plan.validate(safety)) == "OK", "Spherical innerhalb Sicherheitsgrenzen annehmen");
  TargetPositionV2 target;
  assertTrue(plan.targetFor(3, target) && target.panMdeg == 60000 && target.tiltMdeg == -45000 &&
             target.column == 3 && target.row == 0, "Erstes Tilt-Band endet am Pan-Endpunkt");
  assertTrue(plan.targetFor(4, target) && target.panMdeg == -60000 && target.tiltMdeg == 0 &&
             target.column == 0 && target.row == 1, "Naechstes Tilt-Band startet am Pan-Anfang");
  assertTrue(plan.targetFor(12, target) && target.panMdeg == 60000 && target.tiltMdeg == 45000,
             "Letztes Tilt-Band erreicht den Endpunkt");
  FakeHardware runHardware;
  FakeEvents runEvents;
  PanoramaExecutor executor(runHardware, safety, runEvents);
  assertTrue(std::string(executor.loadPlan(plan)) == "PLAN_ACCEPTED" &&
             std::string(executor.start(880, 0)) == "STARTED", "Spherical autonom starten");
  for (uint32_t image = 0; image < 13; image++) completeOneImage(executor, runHardware, image * 100);
  executor.tick(1300);
  assertTrue(runEvents.images == 13 && executor.state() == RUN_COMPLETED,
             "Alle 13 variablen Bandpunkte autonom aufnehmen");
  plan.endTiltMdeg = 51000;
  assertTrue(std::string(plan.validate(safety)) == "TILT_LIMIT", "Spherical respektiert Tilt-Sicherheitsgrenze");
  plan.endTiltMdeg = 45000;
  plan.startPanMdeg = -180000;
  plan.endPanMdeg = 180000;
  plan.startTiltMdeg = 0;
  plan.endTiltMdeg = 0;
  plan.rows = 1;
  plan.columns = 4;
  plan.stepPanMdeg = 90000;
  assertTrue(plan.totalImages() == 4 && plan.targetFor(3, target) && target.panMdeg == 90000,
             "360 Grad vermeiden doppeltes Abschlussbild");

  FakeStream stream;
  FakeHardware hardware;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(110, "LOAD_PLAN", "880|2|1|-60000|-45000|5|3|30000|30000|10|30|50|500|500|0|0|60000|45000"));
  controller.tick(1);
  assertTrue(stream.output.find("|ACK|110|PLAN_ACCEPTED|") != std::string::npos,
             "Sphaerisches 18-Feld-Rezept wird per V2 angenommen");
  stream.feed(frame(112, "GET_PLAN", ""));
  controller.tick(2);
  assertTrue(stream.output.find("|PLAN|880|2|1|") != std::string::npos &&
             stream.output.find("|60000|45000|") != std::string::npos,
             "Sphaerisches Rezept wird fuer Reconnect zurueckgemeldet");
  stream.output.clear();
  stream.feed(frame(111, "REWIND_COLUMNS", "880|1"));
  controller.tick(3);
  assertTrue(stream.output.find("|NACK|111|UNSUPPORTED_MODE|") != std::string::npos,
             "Spaltennavigation ist fuer variable Bandlaengen gesperrt");
}

static void testActiveV2SessionResynchronizesBeforeReconnectHello()
{
  FakeHardware hardware;
  FakeStream stream;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(std::string(1, (char)-6));
  controller.tick(0);
  assertTrue(stream.available() == 1 && stream.output.empty(), "Before first HELLO preserve V1 input for legacy parser");
  stream.read();
  stream.feed(frame(200, "HELLO", "ANDROID"));
  controller.tick(1);
  assertTrue(stream.output.find("|ACK|200|HELLO_ACK|") != std::string::npos, "Initial HELLO acknowledged");
  stream.output.clear();
  // A stray byte after a truncated UART frame must not hide every later HELLO.
  stream.feed(std::string("\r\n%CONNECT\r\n") + frame(201, "HELLO", "ANDROID"));
  controller.tick(2);
  controller.tick(1000);
  assertTrue(stream.output.find("|ACK|201|HELLO_ACK|") != std::string::npos, "Reconnect HELLO survives leading UART noise in active V2 session");
  assertTrue(hardware.targetCalls == 0 && !hardware.camera, "Resynchronization causes no movement or camera pulse");
  stream.output.clear();
  stream.feed(std::string("truncated-tail\r\n") + frame(202, "ESTOP", "0|TEST"));
  controller.tick(1001);
  assertTrue(stream.output.find("|ACK|202|STOPPED|") != std::string::npos && hardware.stopped,
             "Active V2 resynchronization also preserves emergency stop");
}

static void testNavigationPreservesInactiveRuns()
{
  for (int scenario = 0; scenario < 3; scenario++)
  {
    FakeHardware hardware; FakeStream stream; V2Controller controller(stream, hardware);
    controller.begin(0);
    stream.feed(frame(300, "HELLO", "NAVIGATION")); controller.tick(1);
    assertTrue(stream.output.find("|NAVIGATION_CAPABILITIES|1|") != std::string::npos,
               "Firmware meldet Fahrfreigabe fuer inaktive Auftraege");
    stream.feed(frame(301, "LOAD_PLAN", "900|1|0|0|0|1|1|1000|0|10|30|50|500|500|0|0")); controller.tick(2);
    if (scenario == 1) { stream.feed(frame(302, "STOP", "900")); controller.tick(3); }
    if (scenario == 2)
    {
      stream.feed(frame(302, "START", "900")); controller.tick(3);
      int before = hardware.targetCalls;
      stream.feed(frame(303, "MANUAL_MOVE", "1000|1000")); controller.tick(4);
      assertTrue(stream.output.find("|NACK|303|INVALID_STATE|") != std::string::npos && hardware.targetCalls == before,
                 "Pfeilbefehl waehrend Aufnahme wird ohne Laufabbruch verworfen");
      for (uint32_t time = 5; time < 150; time++) { hardware.reached = true; controller.tick(time); }
      stream.feed(frame(308, "GET_STATUS", "")); controller.tick(150);
      assertTrue(stream.output.find("|STATUS|900|COMPLETED|NONE|1|1|") != std::string::npos,
                 "Aufnahme laeuft trotz verworfenem Pfeilbefehl bis Abschluss");
    }
    const char *expected = scenario == 0 ? "|STATUS|900|PLAN_READY|NONE|0|1|" :
                           scenario == 1 ? "|STATUS|900|STOPPED|NONE|0|1|" : "|STATUS|900|COMPLETED|NONE|1|1|";
    stream.output.clear();
    stream.feed(frame(304, "MANUAL_MOVE", "1000|1000")); controller.tick(151);
    assertTrue(stream.output.find("|ACK|304|MOVING|") != std::string::npos,
               "Geladene, gestoppte und abgeschlossene Auftraege erlauben Pfeilfahrt");
    stream.feed(frame(305, "START", "900")); controller.tick(152);
    assertTrue(stream.output.find("|NACK|305|MANUAL_ACTION_ACTIVE|") != std::string::npos,
               "Aufnahmestart darf laufende manuelle Fahrt nicht uebernehmen");
    hardware.reached = true; controller.tick(153);
    assertTrue(stream.output.find(expected) != std::string::npos && !hardware.camera,
               "Manuelle Fahrt erhaelt Laufzustand, Laufkennung und Aufnahmezaehler");
    stream.feed(frame(306, "GET_PLAN", "")); controller.tick(154);
    assertTrue(stream.output.find("|PLAN|900|1|0|0|0|1|1|") != std::string::npos,
               "Rezept bleibt nach manuellem Anfahren unveraendert geladen");
    if (scenario == 0)
    {
      stream.feed(frame(307, "START", "900")); controller.tick(155);
      assertTrue(stream.output.find("|ACK|307|STARTED|") != std::string::npos,
                 "Geladenen Lauf nach Zielankunft normal starten");
    }
  }
}

static void testManualReturnFromTiltLimit()
{
  FakeHardware hardware;
  FakeStream stream;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  hardware.tilt = -47373;
  stream.feed(frame(220, "MANUAL_MOVE", "0|-44000"));
  controller.tick(1);
  assertTrue(stream.output.find("|ACK|220|MOVING|") != std::string::npos, "Rueckfahrt vom ueberfahrenen Tilt-Limit wird angenommen");
  hardware.tilt = -47373; // Real encoder remains outside while the return starts.
  controller.observeTilt(hardware.tilt);
  assertTrue(controller.motionEnabled() && !hardware.stopped, "Grenzbeobachtung darf angenommene Rueckfahrt nicht wieder blockieren");
  assertTrue(controller.limitDrive(true, -47373, -200) == 0 && controller.limitDrive(true, -47373, 200) == 200,
             "Auswaerts am ueberfahrenen Limit sperren, Rueckfahrt erlaubt");
  stream.output.clear();
  stream.feed(frame(221, "MANUAL_MOVE", "0|-47473")); controller.tick(2);
  assertTrue(stream.output.find("|NACK|221|TILT_LIMIT|") != std::string::npos, "Weiter nach aussen wird abgelehnt");
  stream.feed(frame(222, "MANUAL_MOVE", "0|-47273")); controller.tick(3);
  hardware.tilt = -47373; controller.observeTilt(hardware.tilt);
  assertTrue(controller.motionEnabled() && stream.output.find("|ACK|222|MOVING|") != std::string::npos,
             "Auch kleine Rueckschritte ausserhalb der Grenze bleiben fahrbar");
}

static void testDirectionalMotorLimits()
{
  SafetyController safety; safety.enableMotion();
  assertTrue(safety.limitDrive(true, -45000, -255) == 0 && safety.limitDrive(true, 50000, 255) == 0,
             "Motoransteuerung stoppt nach aussen bereits genau an beiden Tilt-Grenzen");
  assertTrue(safety.limitDrive(true, -45000, 200) == 200 && safety.limitDrive(true, 50000, -200) == -200,
             "Motoransteuerung nach innen bleibt an beiden Tilt-Grenzen frei");
  assertTrue(safety.limitDrive(true, -44000, -255) == -255 && safety.limitDrive(true, 49000, 255) == 255,
             "Innerhalb des Bereichs bleibt die gewaehlte Motorleistung unveraendert");
  assertTrue(safety.limitDrive(false, 180000, 255) == 0 && safety.limitDrive(false, 181000, -200) == -200,
             "Pan-Grenze blockiert nur die falsche Richtung");
  assertTrue(safety.validateManualTarget(0, 51000, 0, 50900) == SAFETY_OK &&
             safety.validateManualTarget(0, 51000, 0, 51100) == SAFETY_TILT_LIMIT,
             "Rueckfahrt am anderen Tilt-Limit validiert, keine Grenzerweiterung");
  safety.stopMotion();
  assertTrue(safety.limitDrive(true, 0, 200) == 0, "Stop bleibt unabhaengig von der Richtung wirksam");
}

static void testBootIdentitySurvivesReconnect()
{
  FakeHardware hardware;
  FakeStream stream;
  V2Controller controller(stream, hardware);
  controller.begin(0);
  stream.feed(frame(230, "HELLO", "ANDROID")); controller.tick(1);
  size_t start = stream.output.find("|DEVICE_SESSION|");
  assertTrue(start != std::string::npos, "HELLO publishes firmware boot identity");
  std::string identity = stream.output.substr(start, stream.output.find('|', start + 16) - start);
  stream.output.clear();
  stream.feed(frame(231, "HELLO", "ANDROID")); controller.tick(2);
  assertTrue(stream.output.find(identity + "|") != std::string::npos, "Bluetooth reconnect preserves boot identity");
  stream.output.clear();
  V2Controller restarted(stream, hardware); restarted.begin(3);
  stream.feed(frame(232, "HEARTBEAT", "0")); restarted.tick(4);
  assertTrue(stream.output.find("|DEVICE_SESSION|") != std::string::npos && stream.output.find(identity + "|") == std::string::npos,
             "Firmware reboot publishes changed identity even when Bluetooth stays connected");
  stream.output.clear();
  stream.feed(frame(233, "HEARTBEAT", "0")); restarted.tick(5);
  assertTrue(stream.output.find("|DEVICE_SESSION|") == std::string::npos, "Ordinary heartbeat does not repeat boot notification");
  assertTrue(hardware.targetCalls == 0 && !hardware.camera, "Boot identity checks never move or trigger the camera");
}

static void testTiltZeroKeepsPhysicalLimitsAndSurvivesReboot()
{
  FakeHardware hardware; FakeStream stream; V2Controller controller(stream, hardware);
  controller.begin(0); hardware.tilt=5000;
  stream.feed(frame(240, "SET_TILT_ZERO", "")); controller.tick(1);
  assertTrue(stream.output.find("|ACK|240|TILT_ZERO_SAVED|") != std::string::npos, "Aktuelle Tiltposition als Nullpunkt speicherbar");
  assertTrue(hardware.tilt == 5000 && hardware.actualTiltMdeg() == 0 && hardware.targetCalls == 0,
             "Nullpunkt setzen aendert Anzeige, erzeugt aber keine Bewegung");
  hardware.tilt=5100; stream.feed(frame(240, "SET_TILT_ZERO", "")); controller.tick(2);
  assertTrue(hardware.tiltZero == 5000 && hardware.actualTiltMdeg() == 100,
             "Wiederholte Nullpunktanfrage setzt Referenz nicht erneut anhand spaeterer Messwerte");
  hardware.tilt=5000;
  assertTrue(stream.output.find("|CAPABILITIES|-180000|180000|-50000|45000|1|") != std::string::npos,
             "Nullpunkt verschiebt Grenzzahlen und behaelt physische Endlagen");
  assertTrue(controller.validateLegacyTarget(0, INT32_MIN) == SAFETY_TILT_LIMIT,
             "Extremer Legacy-Rohwinkel kann bei Referenzumrechnung nicht ueberlaufen");
  SafetyController calibratedBounds; calibratedBounds.loadTiltLimits(); calibratedBounds.enableMotion();
  assertTrue(calibratedBounds.limitDrive(true, -50000, -255) == 0 && calibratedBounds.limitDrive(true, -50000, 255) == 255,
             "Richtungsabhaengige Limits gelten auch mit verschobenem Nullpunkt");
  FakeHardware rebootHardware; rebootHardware.tilt=5000; FakeStream rebootStream;
  V2Controller rebooted(rebootStream,rebootHardware); rebooted.begin(2);
  assertTrue(rebootHardware.actualTiltMdeg() == 0, "Firmware-Neustart stellt gespeicherten Tilt-Nullpunkt wieder her");
  stream.output.clear();
  stream.feed(frame(241, "SET_TILT_LIMITS", "-40000|40000")); controller.tick(3);
  assertTrue(stream.output.find("|ACK|241|TILT_LIMITS_SAVED|") != std::string::npos,
             "Kalibrierte Grenzwerte werden in derselben Referenz angenommen");
  stream.output.clear();
  stream.feed(frame(246, "SET_TILT_LIMITS", "-60000|40000")); controller.tick(3);
  assertTrue(stream.output.find("|NACK|246|VALUE_OUT_OF_RANGE|") != std::string::npos,
             "Verschobener Nullpunkt erweitert den absoluten physischen Grenzbereich nicht");
  stream.output.clear();
  stream.feed(frame(242, "RESET_TILT_ZERO", "")); controller.tick(4);
  assertTrue(hardware.tilt == 5000 && hardware.actualTiltMdeg() == 5000 &&
             stream.output.find("|CAPABILITIES|-180000|180000|-35000|45000|1|") != std::string::npos,
             "Werkreferenz stellt Winkel wieder her und behaelt geaenderte physische Limits");
  stream.feed(frame(243, "SET_TILT_LIMITS", "-45000|50000")); controller.tick(5);
  stream.feed(frame(244, "MANUAL_MOVE", "0|6000")); controller.tick(6); stream.output.clear();
  stream.feed(frame(245, "SET_TILT_ZERO", "")); controller.tick(7);
  assertTrue(stream.output.find("|NACK|245|INVALID_STATE|") != std::string::npos,
             "Nullpunkt waehrend einer Fahrt nicht veraenderbar");
}

static void testRealAdapterMapsCalibratedTargetsToEncoder()
{
  extern IPanoramaHardware &testHardwareAdapter();
  extern void loop();
  extern double AbsPos_y, TargetPos_y;
  IPanoramaHardware &adapter=testHardwareAdapter();
  assertTrue(adapter.columnReturnTiltTempo(500)==800,"Echter Hardwareadapter nutzt den konfigurierten Spaltenrueckfahrtwert");
  assertTrue(adapter.incrementalMoveMaximumDurationMs()==3000,"Echter Hardwareadapter nutzt 3000 ms als konfiguriertes Schrittbudget");
  AbsPos_y=5.0;
  assertTrue(adapter.setTiltZeroOffsetMdeg(5000) && adapter.actualTiltMdeg() == 0 && TargetPos_y == 5.0,
             "Echter Hardwareadapter setzt Referenz ohne den gehaltenen Encoderwinkel zu verschieben");
  Wire.raw[1] = (uint16_t)(16383 - lround(5.0 * 16384 / 360)); testMillis += 20; loop();
  adapter.configureMotionLimits(-180000,180000,-50000,45000);
  assertTrue(adapter.setTargetMdeg(0,1000,500,500,false), "Echter Adapter nimmt frisches kalibriertes Ziel an");
  MotionDiagnosticsV2 diagnostics; adapter.getMotionDiagnostics(diagnostics);
  assertTrue(TargetPos_y == 6.0 && diagnostics.tiltTargetMdeg == 1000,
             "Kalibriertes Tiltziel wird fuer Regler zum Rohwinkel und fuer Diagnose zurueckgerechnet");
  adapter.setTiltZeroOffsetMdeg(0); Wire.raw[1]=16383; AbsPos_y=0; TargetPos_y=0;
}

static void testDisconnectedEncodersStopRealMotorLoop()
{
  extern uint8_t encoderReadError[2];
  extern uint32_t encoderReadFailures[2];
  extern void loop();
  extern V2Controller v2Controller;
  extern IPanoramaHardware &testHardwareAdapter();
  v2Controller.begin(1);
  testMillis += 20; loop();
  Serial.feed(frame(501,"HELLO","2")); testMillis += 20; loop();
  Serial.feed(frame(502,"MANUAL_MOVE","5000|5000"));
  // Disconnect while both axes are driven, before the independent 1.5 s
  // no-start timeout of the staged Tilt controller would stop this stub.
  for(int i=0;i<40;i++) { testMillis += 20; loop(); }
  assertTrue(testAnalogOutputs[3] > 0 && testAnalogOutputs[9] > 0,
             "Echter Regler treibt mit gueltigen Encodern beide Motoren an");
  MotionDiagnosticsV2 before, after;
  testHardwareAdapter().getMotionDiagnostics(before);
  Wire.connected[1]=false;
  uint32_t tiltFailures=encoderReadFailures[1];
  testMillis += 20; loop();
  testHardwareAdapter().getMotionDiagnostics(after);
  assertTrue(testAnalogOutputs[3] == 0 && testAnalogOutputs[9] == 0 &&
             !v2Controller.motionEnabled(),
             "Abgezogener Tiltencoder stoppt beide PWM-Ausgaenge im selben Regelzyklus");
  assertTrue(after.sampleSequence == before.sampleSequence,
             "Ungueltiges Encodersample bestaetigt keine Zielankunft");
  assertTrue(encoderReadError[1]==4 && encoderReadFailures[1]==tiltFailures+2 && encoderReadError[0]==0,
             "Sensorprobe unterscheidet fehlgeschlagene I2C-Uebertragung und betroffene Achse");
  Wire.connected[1]=true;
  testMillis += 20; loop();
  assertTrue(testAnalogOutputs[3] == 0 && testAnalogOutputs[9] == 0,
             "Wiederanschliessen startet den alten Fahrbefehl nicht erneut");
  assertTrue(encoderReadError[1]==0 && encoderReadFailures[1]==tiltFailures+2,
             "Gueltige Folgeprobe behaelt die Zahl der beobachteten Sensorfehler");
  Serial.feed(frame(503,"CLEAR_RUN","")); testMillis += 20; loop();
  Wire.connected[0]=false;
  Serial.feed(frame(504,"MANUAL_MOVE","6000|6000")); testMillis += 20; loop();
  assertTrue(testAnalogOutputs[3] == 0 && testAnalogOutputs[9] == 0 &&
             !v2Controller.motionEnabled(),
             "Fehlender Panencoder verhindert bereits den Fahrtbeginn");
  Wire.connected[0]=true; Wire.responseBytes=1;
  Serial.feed(frame(505,"CLEAR_RUN","")); testMillis += 20; loop(); testMillis += 20; loop();
  assertTrue(testAnalogOutputs[3] == 0 && testAnalogOutputs[9] == 0 &&
             !v2Controller.motionEnabled(),
             "Unvollstaendige I2C-Antwort gilt nicht als gueltiger Winkel");
  assertTrue(encoderReadError[0]==11 && encoderReadError[1]==11,
             "Ein einzelnes Antwortbyte wird als unvollstaendige I2C-Leseantwort diagnostiziert");
  Wire.responseBytes=2;
  Serial.feed(frame(506,"CLEAR_RUN","")); testMillis += 20; loop();
}

static void testBlockedEncoderBusRecovery()
{
  extern void loop();
  extern V2Controller v2Controller;
  extern IPanoramaHardware &testHardwareAdapter();
  Wire.raw[0]=Wire.raw[1]=16383;
  v2Controller.begin(1);
  // Rearm the bounded recovery burst only after genuinely healthy feedback.
  for(int i=0;i<60;++i) { testMillis+=20; loop(); }
  Serial.feed(frame(811,"HELLO","2")); testMillis+=20; loop();
  Serial.feed(frame(812,"MANUAL_MOVE","5000|5000"));
  for(int i=0;i<10;++i) { testMillis+=20; loop(); }
  assertTrue(testAnalogOutputs[3]>0 && testAnalogOutputs[9]>0,"Busfehler-Test beginnt waehrend einer echten Motorfahrt");
  MotionDiagnosticsV2 before, after;
  testHardwareAdapter().getMotionDiagnostics(before);
  unsigned int resets=Wire.endCalls;
  Wire.recoveryAlwaysUnpowered=true;
  testSdaHeldLow=true; testBusClockPulses=0; testSdaReleaseAfterPulses=3;
  testMillis+=20; loop();
  testHardwareAdapter().getMotionDiagnostics(after);
  assertTrue(!testSdaHeldLow && Wire.endCalls==resets+1 && after.sampleSequence==before.sampleSequence+1 &&
             v2Controller.motionEnabled(),
             "Blockierte SDA wird ohne Benutzerneustart freigegeben und nur frische Folgeprobe regelt weiter");
  assertTrue(Wire.recoveryAlwaysUnpowered && testBusClockPulses>=3 && testBusClockPulses<=10,
             "Busfreigabe erfolgt ausschliesslich bei abgeschalteten Motorausgaengen und mit begrenzten Takten");
  before=after; testSclHeldLow=true;
  testMillis+=20; loop(); testHardwareAdapter().getMotionDiagnostics(after);
  assertTrue(!v2Controller.motionEnabled() && !testAnalogOutputs[3] && !testAnalogOutputs[9] &&
             after.sampleSequence==before.sampleSequence,
             "Nicht freigebbarer Taktbus beendet den Fahrbefehl ohne fingierte Sensorprobe");
  resets=Wire.endCalls;
  for(int i=0;i<150;++i) { testMillis+=20; loop(); }
  assertTrue(Wire.endCalls==resets+2 && !testAnalogOutputs[3] && !testAnalogOutputs[9],
             "Dauerhaft blockierter Bus erhaelt hoechstens drei zeitlich getrennte Freigabeversuche");
  testSclHeldLow=false; testSdaReleaseAfterPulses=0;
  testMillis+=20; loop();
  assertTrue(!v2Controller.motionEnabled() && !testAnalogOutputs[3] && !testAnalogOutputs[9],
             "Sensorwiederkehr nach gescheiterter Freigabe startet keinen alten Auftrag");
  Serial.feed(frame(813,"CLEAR_RUN","")); testMillis+=20; loop();
  for(int i=0;i<60;++i) { testMillis+=20; loop(); }
  Serial.feed(frame(814,"MANUAL_MOVE","5000|5000"));
  for(int i=0;i<10;++i) { testMillis+=20; loop(); }
  testHardwareAdapter().getMotionDiagnostics(before);
  testSdaHeldLow=true; testBusClockPulses=0; testSdaReleaseAfterPulses=3; Wire.transferUs=2000;
  testMillis+=20; uint32_t started=micros(); loop();
  testHardwareAdapter().getMotionDiagnostics(after);
  assertTrue(uint32_t(micros()-started)<=MOTION_ENCODER_RECOVERY_BUDGET_US &&
             after.sampleSequence==before.sampleSequence && !v2Controller.motionEnabled() &&
             !testAnalogOutputs[3] && !testAnalogOutputs[9],
             "Spaete Folgeantwort ueberschreitet kein Wiederherstellungsbudget und bestaetigt keine Zielankunft");
  Wire.transferUs=0; testSdaHeldLow=false; testSdaReleaseAfterPulses=0;
  Serial.feed(frame(815,"CLEAR_RUN","")); testMillis+=20; loop();
}

class QuietHardware : public FakeHardware
{
public:
  bool candidate = false;
  uint32_t since = 0;
  bool targetQuietSince(uint32_t &value) const { value = since; return candidate; }
};

static void testProfileSettlingUsesOneClockAndProtectsExposure()
{
  QuietHardware hardware; FakeEvents events; SafetyController safety;
  PanoramaExecutor executor(hardware,safety,events);
  PanoramaPlanV2 plan = makePlan(700,1); plan.settleTimeMs=700;
  executor.loadPlan(plan); executor.start(700,0); executor.tick(0);
  hardware.candidate=true; hardware.since=50; executor.tick(50);
  assertTrue(executor.phase()==PHASE_SETTLING && !hardware.camera, "Ruhekandidat startet Beruhigung vor Abschluss der Bestaetigung");
  hardware.reached=true; executor.tick(300); executor.tick(749);
  assertTrue(!hardware.camera, "Gemeinsame Ruhezeit muss vollstaendig verstreichen");
  executor.tick(750);
  assertTrue(hardware.camera, "700 ms ab Ruhekandidat genuegen ohne zusaetzliche 250-ms-Wartephase");
  hardware.reached=false; hardware.candidate=false; executor.tick(751);
  assertTrue(executor.state()==RUN_FAULT && !hardware.camera && hardware.stopped && events.images==0 &&
    std::string(executor.lastError())=="EXPOSURE_UNSTABLE", "Positionsverlust nach Ausloesung gilt nicht als gute Aufnahme");
}

static void testCompletionHoldsAndExplicitStopDisables()
{
  FakeHardware hardware; FakeEvents events; SafetyController safety;
  PanoramaExecutor executor(hardware,safety,events);
  executor.loadPlan(makePlan(701,1)); executor.start(701,0); completeOneImage(executor,hardware,0);
  assertTrue(executor.state()==RUN_COMPLETED && hardware.stopped && !safety.motionEnabled(), "Normale Aufnahme und Laufabschluss bleiben ohne Motorimpulse");
  assertTrue(std::string(executor.stop(701,"USER_STOP"))=="STOPPED" && hardware.stopped && !safety.motionEnabled() &&
    executor.completedImages()==1, "Stop nach Abschluss schaltet Halten ab und behaelt Aufnahmezaehler");
}

static void testManualTempoStopAndPartialRecovery()
{
  FakeHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
  stream.feed(frame(799,"SET_TILT_LIMITS","-45000|50000")); controller.tick(0);
  stream.feed(frame(800,"HELLO","TEST")); controller.tick(1);
  assertTrue(stream.output.find("|MOTION_CAPABILITIES|1|")!=std::string::npos, "Neues Fahrprofil ist explizit angekuendigt");
  stream.feed(frame(801,"MANUAL_MOVE","1000|0|200|350")); controller.tick(2);
  assertTrue(hardware.panTempo==200 && hardware.tiltTempo==350, "Manuelles Tempo gelangt getrennt je Achse zum Profil");
  int calls=hardware.targetCalls;
  stream.feed(frame(802,"MANUAL_MOVE","2000|0|0|350")); controller.tick(3);
  assertTrue(hardware.targetCalls==calls, "Ungueltiges Tempo aendert keine laufende Fahrt");
  stream.feed(frame(803,"STOP","0")); controller.tick(4);
  assertTrue(hardware.stopped && !controller.motionEnabled() && stream.output.find("|ACK|803|STOPPED|")!=std::string::npos,
    "Normaler Stop beendet auch eine manuelle Fahrt ohne Aufnahmeauftrag");
  stream.feed(frame(804,"CLEAR_RUN","")); controller.tick(5);
  hardware.tilt=52000;
  stream.feed(frame(805,"MANUAL_MOVE","1000|51000")); controller.tick(6);
  hardware.reached=true; controller.tick(7);
  assertTrue(hardware.stopped && !controller.motionEnabled(), "Teilweise Rueckfahrt haelt nicht ausserhalb des Limits nach");
  stream.feed(frame(806,"MANUAL_MOVE","1000|50000")); controller.tick(8);
  assertTrue(stream.output.find("|ACK|806|MOVING|")!=std::string::npos, "Naechster einwaertiger Pfeilschritt bleibt nach Teilrueckfahrt moeglich");
  stream.feed(frame(807,"STOP","0")); controller.tick(9);
  stream.feed(frame(808,"CLEAR_RUN","")); controller.tick(10);
  hardware.pan=181000; hardware.tilt=52000; hardware.reached=false;
  stream.feed(frame(809,"MANUAL_MOVE","180500|52000")); controller.tick(11);
  // FakeHardware accepts targets immediately; restore the still-moving feedback.
  hardware.pan=181000; hardware.tilt=52000; controller.observeTilt(hardware.tilt);
  assertTrue(controller.motionEnabled() && stream.output.find("|ACK|809|MOVING|")!=std::string::npos,
    "Beide Achsen ausserhalb: Pan-Rueckfahrt darf unveraenderten Tilt nicht als neuen Limitfehler behandeln");
}

static void testRejectedStopPreservesManualWatchAndCalibrationAfterHold()
{
  FakeHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
  stream.feed(frame(810,"LOAD_PLAN","900|1|0|0|0|1|1|1000|0|10|30|50|500|500|0|0")); controller.tick(1);
  stream.feed(frame(811,"MANUAL_MOVE","1000|0")); controller.tick(2);
  stream.feed(frame(812,"STOP","901")); controller.tick(3);
  stream.feed(frame(813,"START","900")); controller.tick(4);
  assertTrue(stream.output.find("|NACK|812|RUN_ID_MISMATCH|")!=std::string::npos &&
    stream.output.find("|NACK|813|MANUAL_ACTION_ACTIVE|")!=std::string::npos && !hardware.stopped,
    "Abgelehnter Stop entfernt nicht die Ueberwachung der laufenden manuellen Fahrt");
  stream.feed(frame(814,"ESTOP","900|TEST")); controller.tick(5);
  stream.feed(frame(815,"CLEAR_RUN","")); controller.tick(6);
  stream.feed(frame(816,"MANUAL_MOVE","1000|1000")); controller.tick(7);
  hardware.reached=true; controller.tick(8);
  stream.feed(frame(817,"SET_TILT_ZERO","")); controller.tick(9);
  assertTrue(stream.output.find("|ACK|817|TILT_ZERO_SAVED|")!=std::string::npos && !controller.motionEnabled(),
    "Nullpunkt kann nach bestaetigter manueller Ankunft aus ruhigem Halten gesetzt werden");
  stream.feed(frame(818,"RESET_TILT_ZERO","")); controller.tick(10);
}

class FakeBenchHardware : public FakeHardware
{
public:
  int pulses=0;
  bool active=false,recording=false;
  const char *reason="NONE";
  bool startBenchDrive(bool,int,uint32_t,int32_t,uint32_t)
  { ++pulses; active=recording=true; stopped=false; reason="RUNNING"; return true; }
  bool startBenchCruise(bool tiltAxis,int pwm,uint32_t duration,int32_t travel,uint32_t now)
  { return startBenchDrive(tiltAxis,pwm,duration,travel,now); }
  bool getBenchStatus(BenchDriveStatus &value) const
  { BenchDriveStatus status={2,active,recording,reason}; value=status; return true; }
  bool getBenchSample(uint16_t index,BenchDriveSample &value) const
  { if(index>=2) return false; BenchDriveSample sample={uint32_t(index*20),pan,tilt,0,3}; value=sample; return true; }
  bool getBenchOutput(BenchOutputStatus &value) const
  { BenchOutputStatus output={0,-30,0,1,2000,235,1000,true}; value=output; return true; }
  void stopAllMotion() { stopped=true; active=false; } // Keep trace tail state.
};

static void testBenchProtocolIsolation()
{
  FakeHardware normal; FakeStream unsupported; V2Controller normalController(unsupported,normal); normalController.begin(0);
  unsupported.feed(frame(100,"BENCH_PULSE","1|20|1000|2000")); normalController.tick(1);
  assertTrue(unsupported.output.find("|NACK|100|BENCH_UNSUPPORTED|")!=std::string::npos &&
    !normalController.motionEnabled(), "Normaler Hardwareadapter erlaubt keine unregulierte Messfahrt");

  FakeBenchHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
  stream.feed(frame(101,"HELLO","TEST")); controller.tick(1);
  assertTrue(stream.output.find("|BENCH_CAPABILITIES|3|40|3000|5000|20|192|5000|500|500|")!=std::string::npos,
    "Messfaehiger Adapter veroeffentlicht feste Leistungs-, Zeit- und Weggrenzen");
  stream.feed(frame(102,"BENCH_PULSE","1|41|1000|2000")); controller.tick(2);
  assertTrue(hardware.pulses==0 && !controller.motionEnabled(), "Ungueltige Messleistung erreicht Hardware nicht");
  stream.feed(frame(121,"BENCH_PULSE","1|-40|501|2000")); controller.tick(2);
  assertTrue(hardware.pulses==0 && !controller.motionEnabled(), "Protokoll begrenzt hoehere Messleistung auf kurzen Puls");
  hardware.tilt=TILT_HARD_MAX_MDEG-500; stream.feed(frame(103,"BENCH_PULSE","1|20|1000|2000")); controller.tick(3);
  assertTrue(hardware.pulses==0 && stream.output.find("|NACK|103|BENCH_LIMIT|")!=std::string::npos,
    "Messfahrt wird vorab gegen gespeicherte Limits geprueft");
  hardware.tilt=0;
  stream.feed(frame(104,"BENCH_PULSE","1|20|1000|2000")); controller.tick(4);
  stream.feed(frame(104,"BENCH_PULSE","1|20|1000|2000")); controller.tick(5);
  assertTrue(hardware.pulses==1 && controller.motionEnabled(), "Wiederholung desselben Messbefehls startet nur einen Puls");
  stream.feed(frame(120,"GET_BENCH_OUTPUT","")); controller.tick(5);
  assertTrue(stream.output.find("|BENCH_OUTPUT|0|-30|0|1|2000|235|1000|1|")!=std::string::npos && hardware.pulses==1,
    "Ausgangsdiagnose funktioniert waehrend Messfahrt ohne zusaetzliche Bewegung");
  stream.feed(frame(105,"TRIGGER_ONCE","100")); controller.tick(6);
  stream.feed(frame(106,"MANUAL_MOVE","0|1000")); controller.tick(7);
  stream.feed(frame(107,"SET_TILT_ZERO","")); controller.tick(8);
  assertTrue(!hardware.camera && hardware.targetCalls==0 &&
    stream.output.find("|NACK|105|BENCH_ACTIVE|")!=std::string::npos &&
    stream.output.find("|NACK|107|BENCH_ACTIVE|")!=std::string::npos,
    "Messfahrt sperrt Kamera, Zielaenderung und Koordinatenaenderung");
  stream.feed(frame(108,"GET_BENCH_TRACE","0")); controller.tick(9);
  assertTrue(stream.output.find("|NACK|108|BENCH_ACTIVE|")!=std::string::npos,
    "Messspur wird erst nach Ende der Aufzeichnung uebertragen");
  hardware.active=false; controller.tick(10);
  assertTrue(!controller.motionEnabled() && hardware.stopped, "Pulsende entzieht Firmware-Motorfreigabe");
  hardware.recording=false; hardware.reason="TIME_BOUND"; controller.tick(11);
  stream.feed(frame(109,"GET_BENCH_TRACE","0")); controller.tick(12);
  assertTrue(stream.output.find("|BENCH_SAMPLE|0|0|")!=std::string::npos &&
    stream.output.find("|BENCH_SAMPLE|1|20|")!=std::string::npos, "Messspur liefert indizierte Sensor- und PWM-Werte");
  stream.feed(frame(110,"BENCH_PULSE","1|-20|1000|2000")); controller.tick(13);
  controller.tick(1513);
  assertTrue(!hardware.active && !controller.motionEnabled() && hardware.stopped,
    "Fehlender Hostheartbeat beendet aktive Messfahrt unabhaengig von Auftragsregeln");

  FakeBenchHardware cruise; FakeStream cruiseStream; V2Controller cruiseController(cruiseStream,cruise); cruiseController.begin(0);
  cruiseStream.feed(frame(122,"BENCH_CRUISE","1|-31|3000|5000")); cruiseController.tick(1);
  assertTrue(cruise.pulses==0 && !cruiseController.motionEnabled(), "Anlauf-Protokoll erlaubt keine zu hohe laufende Leistung");
  cruiseStream.feed(frame(123,"BENCH_CRUISE","1|-20|3000|5000")); cruiseController.tick(2);
  cruiseStream.feed(frame(123,"BENCH_CRUISE","1|-20|3000|5000")); cruiseController.tick(3);
  assertTrue(cruise.pulses==1 && cruiseController.motionEnabled(), "Anlauf-Protokoll startet bei Wiederholung nur eine Messung");
  cruiseStream.feed(frame(124,"BENCH_PULSE","1|20|1000|3000")); cruiseController.tick(4);
  assertTrue(cruise.pulses==1 && cruiseStream.output.find("|NACK|124|BENCH_ACTIVE|")!=std::string::npos,
    "Aktive Anlaufmessung kann nicht durch weiteren Puls ersetzt werden");
  cruiseStream.feed(frame(125,"ESTOP","0|TEST")); cruiseController.tick(5);
  assertTrue(!cruise.active && !cruiseController.motionEnabled(), "Notstop beendet auch Anlauf-Protokoll sofort");

  FakeBenchHardware stopHardware; FakeStream stopStream; V2Controller stopController(stopStream,stopHardware); stopController.begin(0);
  stopStream.feed(frame(113,"BENCH_PULSE","1|20|1000|2000")); stopController.tick(1);
  stopStream.feed(frame(114,"ESTOP","0|TEST")); stopController.tick(2);
  assertTrue(!stopHardware.active && !stopController.motionEnabled() && stopHardware.stopped,
    "Protokoll-Notstop unterbricht Messpuls unmittelbar");
  stopStream.output.clear();
  stopStream.feed(frame(114,"ESTOP","0|TEST")); stopController.tick(3);
  assertTrue(!stopHardware.active && !stopController.motionEnabled() &&
    stopStream.output.find("|ACK|114|STOPPED|")!=std::string::npos,
    "Identische Notstop-Wiederholung bestaetigt den Halt ohne Bewegung neu zu starten");

  FakeBenchHardware busy; FakeStream busyStream; V2Controller busyController(busyStream,busy); busyController.begin(0);
  busyStream.feed(frame(111,"LOAD_PLAN","900|1|0|0|0|1|1|1000|0|10|30|50|500|500|0|0")); busyController.tick(1);
  busyStream.feed(frame(112,"BENCH_PULSE","1|20|1000|2000")); busyController.tick(2);
  assertTrue(busy.pulses==0 && busyStream.output.find("|NACK|112|INVALID_STATE|")!=std::string::npos,
    "Geladener Auftrag kann nicht durch Messfahrt beeinflusst werden");
}

class FakeConfigHardware : public FakeHardware
{
public:
  MotionSettings settings;
  uint32_t revision=0;
  int applications=0;
  bool failStore=false;
  bool getMotionConfiguration(MotionSettings &s,uint32_t &r) const
  { s=settings; r=revision; return true; }
  const char *applyMotionConfiguration(const MotionSettings &s,uint32_t r)
  {
    if(failStore) return "CONFIG_STORE_FAILED";
    settings=s; revision=r; ++applications; return 0;
  }
};
static void testMotionConfigProtocol()
{
  FakeConfigHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
  uint16_t sequence=100; uint32_t now=0;
  auto command=[&](const char *type,const std::string &fields) {
    stream.output.clear(); stream.feed(frame(++sequence,type,fields)); controller.tick(++now);
  };
  auto stage=[&](uint32_t token,const MotionSettings &settings) {
    for(unsigned int i=0;i<MOTION_PARAMETER_COUNT;++i)
      command("SET_MOTION_PARAMETER",std::to_string(token)+"|"+std::to_string(i)+"|"+std::to_string(settings.values[i]));
  };
  command("GET_MOTION_CONFIG",std::to_string(MP_TRAVEL_POSITION_ENTER_DEG));
  assertTrue(stream.output.find("|tilt.positionToleranceDeg|300|1000|deg|44|1000|300|")!=std::string::npos,
    "Protokoll liest wirksamen Wert mit Einheit, Skalierung und gueltigem Bereich");
  command("BEGIN_MOTION_CONFIG","777|1|0");
  command("SET_MOTION_PARAMETER","778|0|300");
  assertTrue(stream.output.find("|CONFIG_TOKEN|")!=std::string::npos && hardware.revision==0,
    "Falsche Transaktion kann aktive Konfiguration nicht beeinflussen");
  command("SET_MOTION_PARAMETER","777|"+std::to_string(MP_TRAVEL_POSITION_ENTER_DEG)+"|350");
  command("APPLY_MOTION_CONFIG","777|0");
  assertTrue(stream.output.find("|CONFIG_INCOMPLETE|")!=std::string::npos && hardware.applications==0,
    "Unvollstaendige Konfigurationsdatei wird nicht teilweise wirksam");
  command("MANUAL_MOVE","1000|0|500|500");
  assertTrue(stream.output.find("|CONFIG_EDIT_ACTIVE|")!=std::string::npos && hardware.targetCalls==0,
    "Konfigurationsuebertragung kann nicht mit einer neuen Fahrt vermischt werden");
  MotionSettings changed; changed.set(MP_TRAVEL_POSITION_ENTER_DEG,350); stage(777,changed);
  assertTrue(hardware.settings.values[MP_TRAVEL_POSITION_ENTER_DEG]==300,
    "Auch vollstaendig empfangene Stagingwerte bleiben bis zur geprueften Uebernahme unwirksam");
  command("BEGIN_MOTION_CONFIG","777|1|0");
  command("APPLY_MOTION_CONFIG","777|0");
  assertTrue(hardware.applications==1 && hardware.revision==1 && hardware.settings.values[MP_TRAVEL_POSITION_ENTER_DEG]==350,
    "Wiederholter Transaktionsbeginn verliert keine Werte; volle Konfiguration atomar uebernommen");
  command("GET_MOTION_CONFIG","0"); command("APPLY_MOTION_CONFIG","777|0");
  assertTrue(hardware.applications==1 && stream.output.find("|CONFIG_APPLIED|")!=std::string::npos,
    "Verlorene Uebernahmequittung nach Diagnose fuehrt nicht zu erneutem Speichern");
  command("BEGIN_MOTION_CONFIG","888|1|0");
  assertTrue(stream.output.find("|CONFIG_REVISION|")!=std::string::npos,
    "Veraltete Konfigurationsrevision abgelehnt");
  command("BEGIN_MOTION_CONFIG","888|1|1");
  changed.values[MP_TRAVEL_POSITION_EXIT_DEG]=350; stage(888,changed);
  command("APPLY_MOTION_CONFIG","888|1");
  assertTrue(hardware.applications==1 && stream.output.find("|CONFIG_HYSTERESIS|")!=std::string::npos,
    "Ungueltige Beziehungen lassen die bisherige gesamte Konfiguration unveraendert");
  command("CANCEL_MOTION_CONFIG","888"); command("BEGIN_MOTION_CONFIG","999|1|1");
  changed.values[MP_TRAVEL_POSITION_EXIT_DEG]=400; stage(999,changed); hardware.failStore=true;
  command("APPLY_MOTION_CONFIG","999|1");
  assertTrue(hardware.applications==1 && hardware.revision==1 && stream.output.find("|CONFIG_STORE_FAILED|")!=std::string::npos,
    "Speicherfehler wird auch am Protokoll als nicht uebernommene Konfiguration gemeldet");
  hardware.failStore=false; command("ESTOP","0|TEST"); command("APPLY_MOTION_CONFIG","999|1");
  assertTrue(hardware.applications==1 && stream.output.find("|CONFIG_TOKEN|")!=std::string::npos,
    "Stop verwirft Staging, spaetes Uebernahmetelegramm aktiviert keine alten Werte");
  command("CLEAR_RUN",""); command("BEGIN_MOTION_CONFIG","111|1|1");
  now+=16000; command("APPLY_MOTION_CONFIG","111|1");
  assertTrue(stream.output.find("|CONFIG_TOKEN|")!=std::string::npos && hardware.applications==1,
    "Unterbrochene Konfigurationsuebertragung verfaellt ohne automatische Aktivierung");
  command("MANUAL_MOVE","1000|0|500|500"); command("BEGIN_MOTION_CONFIG","222|1|1");
  assertTrue(stream.output.find("|CONFIG_BUSY|")!=std::string::npos && hardware.applications==1,
    "Laufende manuelle Bewegung verhindert neue Konfiguration");
  command("ESTOP","0|TEST");
}

static void testWarningSettingsProtocol()
{
  FakeConfigHardware h;FakeStream s;V2Controller c(s,h);c.begin(0);
  uint16_t seq=800;uint32_t now=0;
  auto send=[&](const char *type,const char *fields) { s.output.clear();s.feed(frame(++seq,type,fields));c.tick(++now); };
  send("HELLO","TEST");
  assertTrue(s.output.find("|RUN_SPEED_CAPABILITIES|2000|")!=std::string::npos && s.output.find("|CAPTURE_WARNING_SETTINGS|500|500|0|")!=std::string::npos,
    "Neue Freigaben melden Lauftempo und unabhängige Warnschwellen");
  MotionSettings before=h.settings;
  send("SET_CAPTURE_WARNING_TOLERANCE","750|1000|0");
  assertTrue(h.revision==1 && h.applications==1 && h.settings.values[MP_CAPTURE_PAN_WARNING_DEG]==750 &&
    h.settings.values[MP_CAPTURE_TILT_WARNING_DEG]==1000 && h.settings.values[MP_TRAVEL_POSITION_ENTER_DEG]==before.values[MP_TRAVEL_POSITION_ENTER_DEG],
    "Warnschwellen werden gemeinsam dauerhaft geändert, Zielregelung bleibt unverändert");
  send("SET_CAPTURE_WARNING_TOLERANCE","750|1000|1");
  assertTrue(h.revision==1 && h.applications==1,"Identische Warnschwellen schreiben EEPROM nicht erneut");
  send("SET_CAPTURE_WARNING_TOLERANCE","750|10001|1");
  assertTrue(h.applications==1 && h.settings.values[MP_CAPTURE_PAN_WARNING_DEG]==750,"Ungültiges Wertepaar ändert nichts");
  send("SET_CAPTURE_WARNING_TOLERANCE","500|500|0");
  assertTrue(h.revision==1,"Veraltete Geräte-Revision überschreibt keine Benutzerwerte");
  h.failStore=true;send("SET_CAPTURE_WARNING_TOLERANCE","500|500|1");
  assertTrue(h.revision==1 && s.output.find("|CONFIG_STORE_FAILED|")!=std::string::npos,"Speicherfehler wird nicht als Erfolg bestätigt");
  h.failStore=false;send("MANUAL_MOVE","1000|0|500|500");send("SET_CAPTURE_WARNING_TOLERANCE","500|500|1");
  assertTrue(h.revision==1 && s.output.find("|CONFIG_BUSY|")!=std::string::npos,"Warnschwellen-Update unterbricht keine Handfahrt");
}

static void testManualStatusSurvivesMissingDebugAndRetries()
{
  FakeHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
  stream.feed(frame(900,"HELLO","MANUAL_STATUS_TEST")); controller.tick(1);
  assertTrue(stream.output.find("|MANUAL_STATUS_CAPABILITIES|1|")!=std::string::npos,
    "Manueller Betriebsstatus wird explizit angekuendigt");
  stream.output.clear();
  stream.feed(frame(901,"MANUAL_MOVE","1000|2000|500|500")); controller.tick(10);
  assertTrue(stream.output.find("|MANUAL_STATUS|901|MOVE|ACTIVE|1000|2000|")!=std::string::npos,
    "Fahrtstatus nennt den angenommenen Befehl und sein Ziel ohne Debug-Abfrage");
  hardware.reached=true; controller.tick(20);
  stream.output.clear();
  stream.feed(frame(902,"GET_STATUS","")); controller.tick(30);
  assertTrue(stream.output.find("|MANUAL_STATUS|901|MOVE|REACHED|1000|2000|10|120000|")!=std::string::npos,
    "Verlorener Fahrabschluss ist nachtraeglich aus Betriebsstatus rekonstruierbar");
  int calls=hardware.targetCalls; stream.output.clear();
  stream.feed(frame(901,"MANUAL_MOVE","1000|2000|500|500")); controller.tick(40);
  stream.feed(frame(903,"GET_STATUS","")); controller.tick(41);
  assertTrue(hardware.targetCalls==calls && stream.output.find("|MANUAL_STATUS|901|MOVE|REACHED|")!=std::string::npos,
    "Spaeter Befehlsretry startet keine Fahrt und verliert den Abschluss nicht");
  for(uint16_t sequence=1000;sequence<1050;sequence++) {
    stream.feed(frame(sequence,"HELLO","RING_OVERWRITE")); controller.tick(sequence);
  }
  stream.output.clear(); stream.feed(frame(1051,"GET_STATUS","")); controller.tick(1100);
  assertTrue(stream.output.find("|MANUAL_STATUS|901|MOVE|REACHED|1000|2000|10|120000|")!=std::string::npos,
    "Abschluss bleibt auch nach Ueberschreiben des Debug-Rings abfragbar");
}

static void testManualStatusInterruptionAndTrigger()
{
  {
    FakeHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
    stream.feed(frame(1100,"HELLO","MANUAL_STATUS_TEST")); controller.tick(1);
    assertTrue(stream.output.find("|MANUAL_STATUS|0|NONE|IDLE|0|0|0|0|")!=std::string::npos,"Neuer Boot hat keinen alten Fahrabschluss");
    stream.feed(frame(1101,"MANUAL_MOVE","1000|2000")); controller.tick(10);
    stream.output.clear(); stream.feed(frame(1102,"STOP","0")); controller.tick(20);
    assertTrue(stream.output.find("|MANUAL_STATUS|1101|MOVE|CANCELLED|1000|2000|10|120000|")!=std::string::npos && hardware.stopped,
      "Normaler Stop bestaetigt Abbruch und behaelt Identitaet der Fahrt");
    stream.feed(frame(1103,"CLEAR_RUN","")); controller.tick(30);
    stream.feed(frame(1104,"MANUAL_MOVE","2000|2000")); controller.tick(40);
    stream.output.clear(); controller.fault("TEST_SENSOR");
    stream.feed(frame(1105,"GET_STATUS","")); controller.tick(50);
    assertTrue(stream.output.find("|MANUAL_STATUS|1104|MOVE|FAULT|")!=std::string::npos && hardware.stopped,
      "Sensorfehler wird nicht als Zielankunft ausgegeben");
  }
  {
    FakeHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
    stream.feed(frame(1110,"HELLO","MANUAL_STATUS_TEST")); controller.tick(1);
    stream.feed(frame(1111,"MANUAL_MOVE","1000|2000")); controller.tick(10);
    stream.output.clear(); controller.tick(120010);
    assertTrue(stream.output.find("|MANUAL_STATUS|1111|MOVE|TIMED_OUT|1000|2000|120000|120000|")!=std::string::npos && hardware.stopped,
      "Zeitablauf beendet Fahrt mit ehrlichem Ergebnis und Dauer");
  }
  {
    FakeHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
    stream.feed(frame(1120,"HELLO","MANUAL_STATUS_TEST")); controller.tick(1);
    stream.feed(frame(1121,"TRIGGER_ONCE","30")); controller.tick(10);
    assertTrue(stream.output.find("|MANUAL_STATUS|1121|TRIGGER|ACTIVE|0|0|0|30|")!=std::string::npos && hardware.camera,
      "Kameraaktion besitzt eigenen aktiven Betriebsstatus");
    controller.tick(40); stream.output.clear(); stream.feed(frame(1122,"GET_STATUS","")); controller.tick(50);
    assertTrue(stream.output.find("|MANUAL_STATUS|1121|TRIGGER|TRIGGERED|0|0|30|30|")!=std::string::npos && !hardware.camera,
      "Verlorenes Kameraereignis laesst Abschluss trotzdem abfragen");
    stream.feed(frame(1123,"TRIGGER_ONCE","30")); controller.tick(60);
    stream.output.clear(); stream.feed(frame(1124,"ESTOP","0|TEST")); controller.tick(70);
    assertTrue(stream.output.find("|MANUAL_STATUS|1123|TRIGGER|CANCELLED|")!=std::string::npos && !hardware.camera,
      "Notstop einer Kameraaktion meldet Abbruch statt erfolgreichen Puls");
  }
  {
    FakeHardware hardware; FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0xfffffff0UL);
    stream.feed(frame(1130,"HELLO","MANUAL_STATUS_TEST")); controller.tick(0xfffffff0UL);
    stream.feed(frame(1131,"TRIGGER_ONCE","30")); controller.tick(0xfffffff0UL);
    controller.tick(14); stream.feed(frame(1132,"GET_STATUS","")); controller.tick(15);
    assertTrue(stream.output.find("|MANUAL_STATUS|1131|TRIGGER|TRIGGERED|0|0|30|30|")!=std::string::npos,
      "Dauer und Abschluss bleiben beim Millis-Ueberlauf richtig");
  }
}

static void testHeldJogProtocol()
{
  class JogHardware : public FakeHardware {
  public:
    bool supportsManualJog() const { return true; }
    bool startManualJog(bool,int,uint16_t) { ++starts; stopped=false; return true; }
    uint16_t manualJogMaximumVelocityMdeg() const { return 5000; }
    uint16_t manualJogMaximumPanVelocityMdeg() const { return 10000; }
    bool startManualJogVelocity(bool,int,uint16_t velocity) { ++starts; lastVelocity=velocity; stopped=false; return true; }
    bool manualJogLimitReached() const { return reached; }
    int starts=0;
    uint16_t lastVelocity=0;
  } hardware;
  FakeStream stream; V2Controller controller(stream,hardware); controller.begin(0);
  stream.feed(frame(1200,"SET_TILT_LIMITS","-45000|30000")); controller.tick(0);
  stream.feed(frame(1201,"HELLO","JOG_TEST")); controller.tick(1);
  assertTrue(stream.output.find("|MANUAL_JOG_CAPABILITIES|1|")!=std::string::npos,"Dauerfahrt hat explizite Freigabe");
  stream.feed(frame(1202,"MANUAL_JOG_START","42|1|-1|300")); controller.tick(10);
  assertTrue(hardware.starts==1 && controller.motionEnabled() && stream.output.find("|MANUAL_STATUS|1202|JOG|ACTIVE|")!=std::string::npos,"Jog startet genau einmal ohne inkrementelle Zielkette");
  stream.feed(frame(1203,"TRIGGER_ONCE","30")); controller.tick(11);
  assertTrue(!hardware.camera,"Dauerfahrt kann keine parallele Kameraausloesung starten");
  stream.feed(frame(1204,"MANUAL_JOG_STOP","41")); controller.tick(12);
  assertTrue(controller.motionEnabled(),"Alter Stop beendet keine neue Geste");
  stream.feed(frame(1205,"MANUAL_JOG_KEEPALIVE","42")); controller.tick(100);
  controller.tick(849); assertTrue(controller.motionEnabled(),"Frisches Haltesignal erhaelt nur die bestehende Fahrt");
  controller.tick(850); assertTrue(hardware.stopped && !controller.motionEnabled(),"Ausbleibendes Tastensignal schaltet unabhaengig vom Bluetooth-Heartbeat ab");
  stream.feed(frame(1206,"MANUAL_JOG_KEEPALIVE","42")); controller.tick(851);
  stream.feed(frame(1207,"MANUAL_JOG_START","42|1|-1|300")); controller.tick(852);
  assertTrue(hardware.starts==1 && !controller.motionEnabled(),"Verspaetete Halte- oder Startpakete koennen beendete Fahrt nicht wieder starten");
  stream.feed(frame(1208,"MANUAL_JOG_STOP","43")); controller.tick(853);
  stream.feed(frame(1209,"MANUAL_JOG_START","43|1|-1|300")); controller.tick(854);
  assertTrue(hardware.starts==1,"Loslassen vor Start verhindert spaeten Anlauf");
  stream.feed(frame(1210,"MANUAL_JOG_START","44|0|1|300")); controller.tick(855);
  stream.feed(frame(1211,"MANUAL_JOG_STOP","44")); controller.tick(856);
  assertTrue(hardware.starts==2 && hardware.stopped && !controller.motionEnabled() &&
    stream.output.find("|MANUAL_STATUS|1210|JOG|CANCELLED|")!=std::string::npos,"Loslassen stoppt beide Ausgaenge ohne einen Fehlerzustand");
  stream.feed(frame(1212,"MANUAL_JOG_START","45|1|1|300")); controller.tick(857);
  hardware.reached=true; controller.tick(858);
  assertTrue(hardware.stopped && !controller.motionEnabled() && stream.output.find("|MANUAL_STATUS|1212|JOG|REACHED|")!=std::string::npos,"Jog endet auch an der inneren Grenzposition");
  hardware.reached=false;
  stream.feed(frame(1214,"MANUAL_JOG_START_VELOCITY","46|1|1|5000")); controller.tick(859);
  assertTrue(hardware.lastVelocity==5000 && controller.motionEnabled(),"Absolutes Halten startet mit 5 Grad/s ohne altes Promillelimit");
  stream.feed(frame(1215,"MANUAL_JOG_STOP","46")); controller.tick(860);
  assertTrue(hardware.stopped && !controller.motionEnabled(),"Absolute Geschwindigkeit besitzt denselben priorisierten Gestenstop");
  int previousStarts=hardware.starts;
  stream.feed(frame(1216,"MANUAL_JOG_START_VELOCITY","47|1|1|6000")); controller.tick(861);
  assertTrue(hardware.starts==previousStarts && stream.output.find("|JOG_SPEED_LIMIT|")!=std::string::npos,"Hoeheres Tempo als die konfigurierbare Grenze startet keinen Motor");
  stream.feed(frame(1217,"MANUAL_JOG_START_VELOCITY","48|0|-1|10000")); controller.tick(862);
  assertTrue(hardware.lastVelocity==10000 && controller.motionEnabled(),"Pan erlaubt getrennte 10 Grad/s bei Tilt-Grenze 5 Grad/s");
  stream.feed(frame(1218,"MANUAL_JOG_STOP","48")); controller.tick(863);
  stream.feed(frame(1213,"SET_TILT_LIMITS","-45000|50000")); controller.tick(864);
}

static void testRealPassiveExposureNoise()
{
  extern IPanoramaHardware &testHardwareAdapter();
  extern double AbsPos_x,AbsPos_y;
  extern bool PanAxisSensorAvailable,TiltAxisSensorAvailable;
  extern void testHardwareSample(uint32_t,bool);
  IPanoramaHardware &adapter=testHardwareAdapter();
  PanAxisSensorAvailable=TiltAxisSensorAvailable=true;
  AbsPos_x=AbsPos_y=0; adapter.setTiltZeroOffsetMdeg(0);
  for(int sample=0;sample<40;++sample) {
    testMillis+=20; AbsPos_x=0.10*std::sin(sample*0.40); AbsPos_y=0.08*std::sin(sample*0.40);
    testHardwareSample(testMillis,false);
    if(sample>=12) assertTrue(adapter.exposurePositionStable(0,0),
      "Belichtung bleibt bei ruhiger Position mit begrenztem Encoderrauschen freigegeben");
  }
  AbsPos_x=1.0;
  for(int sample=0;sample<12;++sample) { testMillis+=20; testHardwareSample(testMillis,false); }
  assertTrue(!adapter.exposurePositionStable(0,0),"Echter Positionsverlust bleibt ein Belichtungsfehler");
  PanAxisSensorAvailable=false;
  assertTrue(!adapter.exposurePositionStable(0,0),"Sensorverlust waehrend Belichtung bleibt sofort gesperrt");
  PanAxisSensorAvailable=true; AbsPos_x=AbsPos_y=0; adapter.setTiltZeroOffsetMdeg(0);
}

static void testIncrementalTargetMissRecovery()
{
  ReturnTempoHardware hardware; FakeStream stream; V2Controller controller(stream,hardware);
  controller.begin(0); stream.feed(frame(680,"HELLO","HOST_TEST")); controller.tick(0);
  stream.feed(frame(681,"LOAD_PLAN","630|1|0|0|0|1|2|0|-1000|10|30|50|500|500|0|5000")); controller.tick(0);
  stream.feed(frame(682,"START","630")); controller.tick(0); hardware.reached=true;
  controller.tick(1); controller.tick(11); controller.tick(41); controller.tick(91); controller.tick(92); controller.tick(93);
  controller.fault("MOTION_TARGET_MISSED"); controller.tick(94);
  stream.feed(frame(683,"GET_STATUS","")); controller.tick(95);
  assertTrue(!controller.motionEnabled() && stream.output.find("|STATUS|630|RUNNING|SETTLING|1|2|")!=std::string::npos &&
    stream.output.find("|STEP_TARGET_MISSED|")!=std::string::npos,
    "Verfehlter inkrementeller Aufnahmepunkt schaltet ab und beruhigt am Istort statt den Lauf vor der Schrittgrenze abzubrechen");
  controller.tick(103); controller.tick(133); controller.tick(183); controller.tick(184);
  stream.feed(frame(684,"GET_STATUS","")); controller.tick(185);
  assertTrue(stream.output.find("|STATUS|630|COMPLETED|NONE|2|2|")!=std::string::npos,
    "Ungefaehrer Aufnahmepunkt wird genau einmal verbucht und der Lauf fortgesetzt");
  // An inaccurate initial/column-return arrival is now a warning too.
  ReturnTempoHardware initial; FakeStream initialStream; V2Controller initialController(initialStream,initial);
  initialController.begin(0); initialStream.feed(frame(685,"HELLO","HOST_TEST")); initialController.tick(0);
  initialStream.feed(frame(686,"LOAD_PLAN","631|1|0|0|0|1|2|0|-1000|10|30|50|500|500|0|5000")); initialController.tick(0);
  initialStream.feed(frame(687,"START","631")); initialController.tick(0);
  initialController.fault("MOTION_TARGET_MISSED"); initialStream.feed(frame(688,"GET_STATUS","")); initialController.tick(1);
  assertTrue(initialStream.output.find("|STATUS|631|RUNNING|SETTLING|")!=std::string::npos &&
    initialStream.output.find("|STEP_TARGET_MISSED|")!=std::string::npos && !initialController.motionEnabled(),
    "Verfehlte Startposition stoppt die Motoren und setzt den Lauf als Warnung fort");
  initialController.tick(11); initialController.tick(41); initialController.tick(91); initialController.tick(92);
  assertTrue(initialStream.output.find("|CAPTURE_WARNING|631|1|8|")!=std::string::npos,
    "Zielwarnung wird mit stabiler Lauf-/Aufnahmeidentitaet statt nur als fluechtiger Status versendet");
  initial.reached=true;initialController.tick(93);initial.reached=true;
  initialController.tick(94);initialController.tick(104);initialController.tick(134);initialController.tick(184);initialController.tick(185);
  initialStream.feed(frame(689,"GET_STATUS",""));initialController.tick(186);
  assertTrue(initialStream.output.find("|STATUS|631|COMPLETED|NONE|2|2|")!=std::string::npos,
    "Nach ungenauer Startposition werden alle folgenden Bilder ohne Abbruch aufgenommen");

  ReturnTempoHardware column; FakeStream columnStream;V2Controller columnController(columnStream,column);columnController.begin(0);
  columnStream.feed(frame(690,"HELLO","HOST_TEST"));columnController.tick(0);
  columnStream.feed(frame(691,"LOAD_PLAN","632|1|0|0|0|2|2|1000|-1000|10|30|50|500|500|0|5000"));columnController.tick(0);
  columnStream.feed(frame(692,"START","632"));columnController.tick(0);column.reached=true;
  columnController.tick(1);columnController.tick(11);columnController.tick(41);columnController.tick(91);columnController.tick(92);columnController.tick(93);
  column.reached=true;columnController.tick(94);columnController.tick(104);columnController.tick(134);columnController.tick(184);columnController.tick(185);columnController.tick(186);
  columnController.fault("MOTION_TARGET_MISSED");columnController.tick(196);columnController.tick(226);columnController.tick(276);columnController.tick(277);columnController.tick(278);
  column.reached=true;columnController.tick(279);columnController.tick(289);columnController.tick(319);columnController.tick(369);columnController.tick(370);
  columnStream.feed(frame(693,"GET_STATUS",""));columnController.tick(371);
  assertTrue(columnStream.output.find("|STATUS|632|COMPLETED|NONE|4|4|")!=std::string::npos && columnStream.output.find("|CAPTURE_WARNING|632|3|8|")!=std::string::npos,
    "Auch Spaltenpositionierung ohne inkrementelles Zeitbudget setzt mit markierter Aufnahme fort");
  columnController.fault("FAULT_PAN_SENSOR");columnStream.feed(frame(694,"GET_STATUS",""));columnController.tick(372);
  assertTrue(columnStream.output.find("|STATUS|632|FAULT|NONE|")!=std::string::npos,
    "Sensorfehler bleiben von Zielqualitaetswarnungen getrennt");

  FakeHardware manual;FakeStream manualStream;V2Controller manualController(manualStream,manual);manualController.begin(0);
  manualStream.feed(frame(695,"HELLO","HOST_TEST"));manualController.tick(0);
  manualStream.feed(frame(696,"MANUAL_MOVE","1000|2000|500|500"));manualController.tick(1);
  manualController.fault("MOTION_TARGET_MISSED");manualStream.feed(frame(697,"GET_STATUS",""));manualController.tick(2);
  assertTrue(manual.stopped && !manualController.motionEnabled() && manualStream.output.find("|MANUAL_STATUS|696|MOVE|TARGET_MISSED|")!=std::string::npos &&
    manualStream.output.find("|STATUS|0|IDLE|")!=std::string::npos,
    "Manuell verfehltes Ziel ist ein abgeschalteter Warnungsabschluss ohne FAULT");
  manualStream.feed(frame(698,"MANUAL_MOVE","2000|2000|500|500"));manualController.tick(3);
  assertTrue(manualController.motionEnabled() && manualStream.output.find("|MANUAL_STATUS|698|MOVE|ACTIVE|")!=std::string::npos,
    "Nach Zielwarnung bleibt eine neue manuelle Fahrt ohne Quittierung moeglich");
}

static void testRealAdapterPanoramaGrid()
{
  extern IPanoramaHardware &testHardwareAdapter();
  extern void testHardwareSample(uint32_t,bool);
  extern double AbsPos_x,AbsPos_y;
  extern bool PanAxisSensorAvailable,TiltAxisSensorAvailable;
  IPanoramaHardware &adapter=testHardwareAdapter();
  PanAxisSensorAvailable=TiltAxisSensorAvailable=true;
  AbsPos_x=AbsPos_y=0; adapter.setTiltZeroOffsetMdeg(0); adapter.configureMotionLimits(-180000,180000,-45000,30000);
  for(int i=0;i<20;++i) { testMillis+=20; testHardwareSample(testMillis,false); }
  SafetyController safety; FakeEvents events; PanoramaExecutor executor(adapter,safety,events);
  PanoramaPlanV2 plan=makePlan(620,5); plan.rows=8; plan.startPanMdeg=4000; plan.startTiltMdeg=10000;
  plan.stepPanMdeg=2062; plan.stepTiltMdeg=-2000; plan.settleTimeMs=700; plan.triggerPulseMs=30; plan.exposureWaitMs=500;
  plan.panSpeedPermille=500; plan.tiltSpeedPermille=500;
  executor.loadPlan(plan); executor.start(plan.runId,testMillis);
  double p[2]={0,0},v[2]={0,0};
  for(int sample=0;sample<16000 && executor.state()!=RUN_COMPLETED && executor.state()!=RUN_FAULT;++sample) {
    bool exposing=executor.phase()==PHASE_TRIGGER_ACTIVE || executor.phase()==PHASE_EXPOSURE_WAIT || executor.phase()==PHASE_COMMIT_IMAGE;
    AbsPos_x=p[0]+(exposing ? .10*std::sin(sample*.4) : 0);
    AbsPos_y=p[1]+(exposing ? .08*std::sin(sample*.4) : 0);
    testMillis+=20; testHardwareSample(testMillis,safety.motionEnabled());
    if(adapter.motionFault()) {
      MotionDiagnosticsV2 d; adapter.getMotionDiagnostics(d);
      std::cerr<<"Grid fault target="<<d.panTargetMdeg<<","<<d.tiltTargetMdeg<<" actual="<<adapter.actualPanMdeg()<<","<<adapter.actualTiltMdeg()<<std::endl;
      executor.fault(adapter.motionFault());
    }
    executor.tick(testMillis);
    MotionTelemetryV2 motion; adapter.getMotionTelemetry(motion);
    int pwm[2]={(int)motion.panSignedPwm,(int)motion.tiltSignedPwm};
    for(int a=0;a<2;++a) {
      double staticPwm=a==0 ? 35.0 : pwm[a]>=0 ? 28.0 : 34.0;
      if(std::abs(v[a])<.05 && std::abs(pwm[a])<staticPwm) { v[a]=0; continue; }
      int direction=v[a]==0 ? (pwm[a]>=0 ? 1 : -1) : (v[a]>0 ? 1 : -1);
      double friction=a==0 ? 26.0 : direction>0 ? 22.0 : 19.0;
      double next=v[a]+(pwm[a]-direction*friction-v[a]/1.3)*.02/.18;
      v[a]=next*direction<0 ? 0 : next; p[a]+=v[a]*.02;
    }
  }
  std::cerr<<"Real adapter grid captures="<<executor.completedImages()<<" error="<<executor.lastError()<<std::endl;
  assertTrue(executor.state()==RUN_COMPLETED && events.images==40 && executor.completedImages()==40,
    "Echter Hardwareadapter beendet Spaltenpanorama mit 40 Aufnahmen trotz Encoderrauschen waehrend Belichtung");
  assertTrue(!safety.motionEnabled() && !testAnalogOutputs[3] && !testAnalogOutputs[9],"Panoramatest endet ohne aktive Motorfreigabe");
  adapter.stopAllMotion(); AbsPos_x=AbsPos_y=0; adapter.setTiltZeroOffsetMdeg(0);
}

static void testWideAngleStepBudget()
{
  extern IPanoramaHardware &testHardwareAdapter();
  extern void testHardwareSample(uint32_t,bool);
  extern double AbsPos_x,AbsPos_y;
  extern bool PanAxisSensorAvailable,TiltAxisSensorAvailable;
  IPanoramaHardware &adapter=testHardwareAdapter();
  PanAxisSensorAvailable=TiltAxisSensorAvailable=true;adapter.stopAllMotion();AbsPos_x=AbsPos_y=0;
  MotionSettings warningSettings;uint32_t warningRevision;adapter.getMotionConfiguration(warningSettings,warningRevision);
  AbsPos_x=warningSettings.number(MP_CAPTURE_PAN_WARNING_DEG)-.001;
  AbsPos_y=warningSettings.number(MP_CAPTURE_TILT_WARNING_DEG)-.001;
  assertTrue(adapter.captureTargetErrorFlags(0,0)==0,"Abweichungen unter separaten Warnschwellen erzeugen keine Winkelwarnung");
  AbsPos_x+=.002;AbsPos_y+=.002;
  assertTrue(adapter.captureTargetErrorFlags(0,0)==3,"Ueberschreitung beider Warnschwellen markiert beide Achsen unabhaengig von Zieltoleranzen");
  AbsPos_x=AbsPos_y=0;
  adapter.setTiltZeroOffsetMdeg(0);adapter.configureMotionLimits(-180000,180000,-45000,30000);
  for(int i=0;i<40;++i) {testMillis+=20;testHardwareSample(testMillis,false);}
  MotionSettings original,wide;uint32_t revision;adapter.getMotionConfiguration(original,revision);wide=original;
  wide.values[MP_INCREMENTAL_MOVE_MAX_DURATION_MS]=3000;
  wide.values[MP_TILT_MAXIMUM_VELOCITY]=5000;
  // The pre-fix firmware has no automatic option and reproduces a 3-s cut.
  for(unsigned int i=0;i<MOTION_PARAMETER_COUNT;++i)
    if(!strcmp(motionParameter(i).name,"panorama.incrementalMoveAutomaticDuration")) wide.values[i]=1;
  assertTrue(!adapter.applyMotionConfiguration(wide,revision+1),"35-mm-Zeitmodelltest startet mit bestaetigter ruhiger Konfiguration");
  assertTrue(adapter.setTargetMdeg(0,-32582,1000,1000,false),"Originaler 35-mm-Tilt-Schritt wird vom echten Adapter geplant");
  uint32_t budget=adapter.incrementalMoveMaximumDurationMs();
  std::cerr<<"35mm tilt step=32.582 budgetMs="<<budget<<std::endl;
  assertTrue(budget>=15000 && budget<=30000,"32.582-Grad-Schritt bekommt physikalische Fahrzeit mit Reserve statt 3-s-Abschaltung");
  adapter.stopAllMotion();
  assertTrue(adapter.setTargetMdeg(0,-32582,1000,500,false) && adapter.incrementalMoveMaximumDurationMs()>budget,
    "Halbes Tempo vergroessert die Zeitgrenze statt die Strecke abzukuerzen");
  adapter.stopAllMotion();
  // The real adapter uses the shared Tilt velocity ramp for panorama steps.
  wide.values[MP_JOG_ACCELERATION]=100;
  const char *accelError=adapter.applyMotionConfiguration(wide,revision+2);
  bool accelAccepted=!accelError && adapter.setTargetMdeg(0,-32582,1000,1000,false);
  std::cerr<<"Slow acceleration accepted="<<accelAccepted<<" error="<<(accelError ? accelError : "none")<<" budgetMs="<<adapter.incrementalMoveMaximumDurationMs()<<std::endl;
  assertTrue(accelAccepted && adapter.incrementalMoveMaximumDurationMs()>budget,
    "Geringere Beschleunigung erhoeht die physikalische Fahrzeit");
  adapter.stopAllMotion();wide.values[MP_MOVE_AUTO_MAX_MS]=5000;
  assertTrue(!adapter.applyMotionConfiguration(wide,revision+3) && adapter.setTargetMdeg(0,-32582,1000,1000,false) && adapter.incrementalMoveMaximumDurationMs()==5000,
    "Explizite automatische Obergrenze bleibt trotz langer Strecke bindend");
  adapter.stopAllMotion();wide.values[MP_MOVE_AUTO_DURATION]=0;
  assertTrue(!adapter.applyMotionConfiguration(wide,revision+4) && adapter.setTargetMdeg(0,-32582,1000,1000,false) && adapter.incrementalMoveMaximumDurationMs()==3000,
    "Ausgeschaltete Automatik erhaelt die bisherige feste Schrittgrenze exakt");
  adapter.stopAllMotion();
  assertTrue(!adapter.applyMotionConfiguration(original,revision+5),"Zeitmodelltest stellt vorherige Konfiguration wieder her");
}

int main(int argc,char **argv)
{
  extern int runMotionTests();
  extern int runMotionConfigTests();
  extern int runPanoramaNavigationRegressions();
  extern int runPanoramaStartRegressions();
  extern int runDownBrakeRegression();
  extern int runPanReturnLoadRegressions();
  extern int runColumnArrivalRegressions();
  extern int runBufferedPanTraceTests();
  extern int runPanArrivalProgressRegression();
  extern int runPanStopModelRegressions();
  if(argc>1 && std::string(argv[1])=="--wide-tilt") { testWideAngleStepBudget();std::cout<<"OK wide tilt: "<<tests<<std::endl;return 0; }
  if(argc>1 && std::string(argv[1])=="--pan-stop-model") { tests+=runPanStopModelRegressions(); std::cout<<"OK pan model: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--pan-progress-band") { tests+=runPanArrivalProgressRegression(); std::cout<<"OK pan progress: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--buffered-trace") { tests+=runBufferedPanTraceTests(); std::cout<<"OK buffered trace: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--column-arrival") { tests+=runColumnArrivalRegressions(); std::cout<<"OK column arrival: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--pan-return-load") { tests+=runPanReturnLoadRegressions(); std::cout<<"OK pan return load: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--brake-window") { tests+=runDownBrakeRegression(); std::cout<<"OK brake: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--run-start") { tests+=runPanoramaStartRegressions(); std::cout<<"OK start: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--navigation") { tests+=runPanoramaNavigationRegressions(); std::cout<<"OK navigation: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--exposure-noise") { testRealPassiveExposureNoise(); std::cout<<"OK exposure: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--grid") { testRealAdapterPanoramaGrid(); std::cout<<"OK grid: "<<tests<<std::endl; return 0; }
  if(argc>1 && std::string(argv[1])=="--step-miss") { testIncrementalTargetMissRecovery(); std::cout<<"OK step: "<<tests<<std::endl; return 0; }
  tests+=runPanoramaNavigationRegressions();
  tests+=runDownBrakeRegression();
  tests+=runPanoramaStartRegressions();
  tests+=runPanReturnLoadRegressions();
  tests+=runColumnArrivalRegressions();
  tests+=runBufferedPanTraceTests();
  tests+=runPanArrivalProgressRegression();
  tests+=runPanStopModelRegressions();
  testRealPassiveExposureNoise();
  testRealAdapterPanoramaGrid();
  testIncrementalTargetMissRecovery();
  testWideAngleStepBudget();
  testWarningSettingsProtocol();
  tests += runMotionConfigTests();
  testManualStatusSurvivesMissingDebugAndRetries();
  testManualStatusInterruptionAndTrigger();
  testHeldJogProtocol();
  testMotionConfigProtocol();
  testSeparateColumnReturnTempo();
  testIncrementalMoveDeadline();
  testPassiveExposureDoesNotReconfirmArrival();
  tests += runMotionTests();
  testBenchProtocolIsolation();
  testDisconnectedEncodersStopRealMotorLoop();
  testBlockedEncoderBusRecovery();
  testRealAdapterMapsCalibratedTargetsToEncoder();
  testTiltZeroKeepsPhysicalLimitsAndSurvivesReboot();
  testBootIdentitySurvivesReconnect();
  testManualReturnFromTiltLimit();
  testNavigationPreservesInactiveRuns();
  testDirectionalMotorLimits();
  testActiveV2SessionResynchronizesBeforeReconnectHello();
  testPauseCompletesCurrentImage();
  testStopIsImmediate();
  testMovementTimeoutStopsMotion();
  testTargetArrivalRequiresFreshStableSamples();
  testSmallManualTargetNeedsMeasuredProgress();
  testDebugEventRingIsBounded();
  testSettlingRevalidatesTargetAndKeepsMovementTimeout();
  testRewindCreatesRevisionWithoutDecrementingCaptureCount();
  testRunningRewindWaitsForCurrentImageAndCombinesClicks();
  testRunningRewindProtocolAcknowledgesAndPublishesSegment();
  testForwardColumnsSkipsRemainderAndKeepsCaptureCount();
  testPausedForwardCombinesClicksAndStopsAtLastColumn();
  testRunningForwardProtocolPublishesSegment();
  testTimingUpdateStartsWithNextImage();
  testTimingUpdateProtocolPublishesAppliedPlan();
  testTiltLimitRejectsWholePlan();
  testNanoPlanImageLimit();
  testProtocolRetryIsIdempotent();
  testSafetyBoundariesAndFragmentation();
  testPausedManualCaptureAndProtocolRewind();
  testAutonomousTimelapseSurvivesReconnect();
  testStoppedTimelapseCanBeCleared();
  testSphericalBandsAndProtocol();
  testTiltLimitsCanBeSavedReadAndRejectUnsafeChanges();
  testProfileSettlingUsesOneClockAndProtectsExposure();
  testCompletionHoldsAndExplicitStopDisables();
  testManualTempoStopAndPartialRecovery();
  testRejectedStopPreservesManualWatchAndCalibrationAfterHold();
  std::cout << "OK: " << tests << " Firmware-Executor-Tests erfolgreich." << std::endl;
  return 0;
}





