#include "MotionController.h"
#include "MotionConfigStore.h"
#include "DeviceRevision22Fixture.h"
#include "DeviceRevision23Fixture.h"
#include "DeviceRevision25Fixture.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
static int checks=0;
static void check(bool value,const char *message)
{ ++checks; if(!value) { std::cerr<<"FEHLER MotionConfig: "<<message<<std::endl; std::exit(1); } }
class FakeConfigStorage : public IMotionConfigStorage
{
public:
  StoredMotionConfig slots[2]; bool failWrite,tornWrite;
  FakeConfigStorage():failWrite(false),tornWrite(false) { memset(slots,0,sizeof(slots)); }
  bool read(uint8_t slot,StoredMotionConfig &r) { r=slots[slot]; return true; }
  bool write(uint8_t slot,const StoredMotionConfig &r)
  {
    if(failWrite) return false;
    if(tornWrite) memcpy(&slots[slot],&r,sizeof(r)/2); else slots[slot]=r;
    return true;
  }
};

int runMotionConfigTests()
{
  MotionSettings defaults;
  check(!defaults.validate(),"Katalog-Standardwerte bilden eine gueltige Gesamtkonfiguration");
  check(defaults.values[MP_COLUMN_RETURN_TEMPO]==800 && defaults.set(MP_COLUMN_RETURN_TEMPO,0),
    "Spaltenrueckfahrt separat mit 800 Promille oder unveraendertem Zeilentempo konfigurierbar");
  defaults.values[MP_COLUMN_RETURN_TEMPO]=800;
  check(defaults.values[MP_INCREMENTAL_MOVE_MAX_DURATION_MS]==3000 &&
    !defaults.set(MP_INCREMENTAL_MOVE_MAX_DURATION_MS,0) && !defaults.set(MP_INCREMENTAL_MOVE_MAX_DURATION_MS,120001),
    "Schrittbudget 3000 ms ist aenderbar, aber weder null noch ueber absolutem Bewegungsbudget");
  check(defaults.number(MP_TRAVEL_POSITION_ENTER_DEG)==0.30 && defaults.values[MP_FINE_ENABLED]==0,
    "Tilt-Toleranz bleibt 0.30 Grad und ungepruefte Feinannaeherung standardmaessig aus");
  check(!defaults.set(MOTION_PARAMETER_COUNT,1) && !defaults.set(MP_TRAVEL_PWM_MAX,50001) &&
    !defaults.set(MP_TRAVEL_DOWN_START_PWM_MAX,50001) && !defaults.set(MP_TRAVEL_DOWN_I_MAX,25001),
    "Unbekannte Parameter und ueberschrittene Stellgrenzen abgelehnt");
  check(defaults.values[MP_TRAVEL_PWM_MAX]==40000 && defaults.values[MP_TRAVEL_DOWN_START_PWM_MAX]==34000 &&
    defaults.values[MP_TRAVEL_DOWN_I_MAX]==12000,"Neue Tilt-Reserve erhoeht keine vorhandenen Standardleistungen");
  MotionSettings invalid=defaults;
  invalid.values[MP_PAN_TARGET_PREDICTION_SAMPLES]=3;
  check(invalid.validate()!=0,"Bremsmodell verlangt ausgeschaltet oder mindestens fuenf Proben");
  invalid=defaults;
  invalid.values[MP_TRAVEL_POSITION_EXIT_DEG]=invalid.values[MP_TRAVEL_POSITION_ENTER_DEG];
  check(invalid.validate()!=0,"Fehlende Hysterese wird nicht teilweise uebernommen");
  invalid=defaults; invalid.values[MP_FINE_STOP_ERROR_DEG]=300;
  check(invalid.validate()!=0,"Fein-Endschwelle muss unter der normalen Fehlertoleranz liegen");
  invalid=defaults; invalid.values[MP_FINE_ENABLED]=1; invalid.values[MP_FINE_MAX_PASSES]=0;
  check(invalid.validate()!=0,"Aktivierte Feinannaeherung ohne begrenzten einzelnen Durchlauf abgelehnt");

  {
    FakeConfigStorage oldStorage;MotionConfigStore oldStore(oldStorage);StoredMotionConfig &old=oldStorage.slots[0];
    old.magic=0x4D4F5431UL;old.schema=1;old.count=128;old.revision=25;
    memcpy(old.values,deviceRevision25,sizeof(deviceRevision25));old.values[128]=(int32_t)deviceRevision25Crc;
    MotionSettings migrated;uint32_t loadedRevision=0;
    check(oldStore.load(migrated,loadedRevision) && loadedRevision==25 &&
      !memcmp(migrated.values,deviceRevision25,sizeof(deviceRevision25)) && migrated.values[MP_CAPTURE_SETTLE_WAIT_MS]==15000,
      "Heutiges 128-Werte-Profil bleibt unveraendert, neue passive Wartezeit erhaelt ihren Standardwert");
    check(oldStore.save(migrated,26) && oldStore.load(migrated,loadedRevision) && loadedRevision==26 &&
      !memcmp(migrated.values,deviceRevision25,sizeof(deviceRevision25)),"Erweiterter Datensatz wird ohne Verlust gespeichert und zurueckgelesen");
  }
  FakeConfigStorage storage; MotionConfigStore store(storage);
  MotionSettings read; uint32_t revision=0;
  {
    FakeConfigStorage oldStorage;MotionConfigStore oldStore(oldStorage);StoredMotionConfig &old=oldStorage.slots[0];
    old.magic=0x4D4F5431UL;old.schema=1;old.count=124;old.revision=23;
    memcpy(old.values,deviceRevision23,sizeof(deviceRevision23));old.values[124]=(int32_t)deviceRevision23Crc;
    check(oldStore.load(read,revision) && revision==23 && !memcmp(read.values,deviceRevision23,sizeof(deviceRevision23)) &&
      read.values[MP_CAPTURE_PAN_WARNING_DEG]==500 && read.values[MP_CAPTURE_TILT_WARNING_DEG]==500 && read.values[MP_MOVE_AUTO_DURATION]==1,
      "124-Werte-Geraeteprofil migriert verlustfrei mit Zeitautomatik und unabhaengigen Warnschwellen");
    check(oldStore.save(read,24) && oldStore.load(read,revision) && revision==24,"Erweiterte 528-Byte-Konfiguration speichert in unveraenderten getrennten EEPROM-Slots");
    oldStorage.slots[1].checksum^=1;
    check(oldStore.load(read,revision) && revision==23 && !memcmp(read.values,deviceRevision23,sizeof(deviceRevision23)),"Beschaedigte Erweiterung faellt auf bestaetigten 124-Werte-Datensatz zurueck");
  }
  check(!store.load(read,revision),"Leerer Speicher wird nicht als gespeicherte Benutzerkonfiguration ausgegeben");
  {
    FakeConfigStorage oldStorage;MotionConfigStore oldStore(oldStorage);StoredMotionConfig &old=oldStorage.slots[0];
    old.magic=0x4D4F5431UL;old.schema=1;old.count=120;old.revision=22;
    memcpy(old.values,deviceRevision22,sizeof(deviceRevision22));old.values[120]=(int32_t)deviceRevision22Crc;
    check(oldStore.load(read,revision) && revision==22 && !memcmp(read.values,deviceRevision22,sizeof(deviceRevision22)) &&
      read.values[MP_MOVE_AUTO_DURATION]==0 && read.values[MP_MOVE_DURATION_FACTOR]==1500 && read.values[MP_MOVE_DURATION_RESERVE_MS]==1000 && read.values[MP_MOVE_AUTO_MAX_MS]==60000,
      "Echtes 120-Werte-Geraeteprofil bleibt vollstaendig erhalten; Zeitautomatik ist beim Upgrade zunaechst aus");
    read.values[MP_MOVE_AUTO_DURATION]=1;
    check(oldStore.save(read,23) && oldStore.load(read,revision) && revision==23 && read.values[MP_MOVE_AUTO_DURATION]==1,
      "124-Werte-Profil aktiviert die Zeitautomatik erst nach verifiziertem Speichern");
    oldStorage.slots[1].checksum^=1;
    check(oldStore.load(read,revision) && revision==22 && !memcmp(read.values,deviceRevision22,sizeof(deviceRevision22)) && read.values[MP_MOVE_AUTO_DURATION]==0,
      "Defekte neue Konfiguration faellt verlustfrei auf das Originalprofil zurueck");
  }
  {
    FakeConfigStorage oldStorage; MotionConfigStore oldStore(oldStorage);StoredMotionConfig &old=oldStorage.slots[0];
    old.magic=0x4D4F5431UL;old.schema=1;old.count=117;old.revision=21;
    memcpy(old.values,defaults.values,117*sizeof(int32_t));old.values[MP_POSITION_ENTER_DEG]=300;
    old.values[MP_POSITION_EXIT_DEG]=400;old.values[MP_PAN_TARGET_COAST_DECELERATION]=60000;
    old.values[117]=0x3842AACC; // Independent Python zlib CRC of complete 2.4.66 prefix.
    check(oldStore.load(read,revision) && revision==21 && !memcmp(read.values,old.values,117*sizeof(int32_t)) &&
      read.values[MP_PAN_TARGET_PREDICTION_SAMPLES]==0 && read.values[MP_PAN_TARGET_COAST_DELAY]==0 &&
      read.values[MP_PAN_TARGET_MIN_APPROACH_VELOCITY]==0,
      "2.4.66-Migration bewahrt alle 117 Benutzerwerte und ergaenzt drei Modellparameter");
    check(oldStore.save(read,22) && oldStore.load(read,revision) && revision==22,"Modell-Konfiguration speichert verifiziert neben bisheriger Kopie");
    oldStorage.slots[1].checksum^=1;
    check(oldStore.load(read,revision) && revision==21 && read.values[MP_PAN_TARGET_COAST_DECELERATION]==60000,
      "Beschaedigte Modell-Konfiguration faellt auf bestaetigte bisherige Werte zurueck");
  }
  {
    FakeConfigStorage oldStorage; MotionConfigStore oldStore(oldStorage);
    StoredMotionConfig &old=oldStorage.slots[0];
    old.magic=0x4D4F5431UL; old.schema=1; old.count=115; old.revision=18;
    memcpy(old.values,defaults.values,115*sizeof(int32_t));
    old.values[MP_PAN_JOG_P_GAIN]=900; old.values[MP_PAN_TARGET_COAST_DECELERATION]=100000;
    // Independent Python zlib CRC of the 2.4.65 prefix with custom values.
    old.values[115]=(int32_t)0xAA77D843UL;
    check(oldStore.load(read,revision) && revision==18 && !memcmp(read.values,old.values,115*sizeof(int32_t)) &&
      read.values[MP_PAN_TARGET_RECOVERY_I_GAIN]==20000 && read.values[MP_PAN_TARGET_RECOVERY_DELAY_MS]==200,
      "2.4.65-Migration bewahrt alle 115 Benutzerwerte und ergaenzt nur Pan-Lastausgleich");
    check(oldStore.save(read,19) && oldStore.load(read,revision) && revision==19 && read.values[MP_PAN_JOG_P_GAIN]==900,
      "117-Parameter-Konfiguration speichert verifiziert neben dem alten Datensatz");
    oldStorage.slots[1].checksum^=1;
    check(oldStore.load(read,revision) && revision==18 && read.values[MP_PAN_TARGET_COAST_DECELERATION]==100000,
      "Beschaedigte neue Kopie faellt auf die bestaetigte 115-Parameter-Konfiguration zurueck");
  }
  {
    FakeConfigStorage oldStorage; MotionConfigStore oldStore(oldStorage);
    StoredMotionConfig &old=oldStorage.slots[0];
    old.magic=0x4D4F5431UL; old.schema=1; old.count=108; old.revision=14;
    memcpy(old.values,defaults.values,108*sizeof(int32_t)); old.values[MP_PAN_JOG_P_GAIN]=900;
    old.values[MP_MANUAL_TILT_DOWN_COAST_DECELERATION]=23000;
    // Independent Python zlib CRC of the 2.4.61 prefix and custom Pan gain.
    old.values[108]=(int32_t)0xD79B285EUL;
    check(oldStore.load(read,revision) && revision==14 && read.values[MP_PAN_JOG_P_GAIN]==900 &&
      read.values[MP_PAN_TARGET_COAST_DECELERATION]==150000 && read.values[MP_CAPTURE_SPAN_DEG]==250,
      "2.4.61-Migration bewahrt alle 108 Benutzerwerte und ergaenzt nur die neuen Parameter");
    check(oldStore.save(read,15) && oldStore.load(read,revision) && revision==15 && read.values[MP_PAN_JOG_P_GAIN]==900,
      "Erweiterte vollstaendige Konfiguration laesst sich neben der bisherigen Kopie verifiziert speichern");
    oldStorage.slots[1].checksum^=1;
    check(oldStore.load(read,revision) && revision==14 && read.values[MP_PAN_JOG_P_GAIN]==900,
      "Beschaedigte erweiterte Kopie faellt auf die gueltige 108-Parameter-Konfiguration zurueck");
  }
  {
    FakeConfigStorage legacyStorage; MotionConfigStore legacyStore(legacyStorage);
    StoredMotionConfig &old=legacyStorage.slots[0];
    old.magic=0x4D4F5431UL; old.schema=1; old.count=85; old.revision=7;
    memcpy(old.values,defaults.values,85*sizeof(int32_t));
    old.values[MP_TRAVEL_POSITION_ENTER_DEG]=350;
    // CRC32 of the actual 2.4.36 schema/count/revision/default-value layout,
    // independently generated with Python zlib; the added setting is absent.
    old.values[85]=0x0517DFCC;
    check(legacyStore.load(read,revision) && revision==7 && read.values[MP_TRAVEL_POSITION_ENTER_DEG]==350 &&
      read.values[MP_COLUMN_RETURN_TEMPO]==800,"2.4.36-Migration behaelt Benutzerwerte und Revision, ergaenzt nur Rueckfahrtempo");
    old.values[MP_TRAVEL_POSITION_ENTER_DEG]++;
    check(!legacyStore.load(read,revision),"Auch alter Datensatz braucht seine unveraenderte CRC; keine ungepruefte Migration");
    old.values[MP_TRAVEL_POSITION_ENTER_DEG]--;
    MotionSettings upgraded=defaults; upgraded.values[MP_COLUMN_RETURN_TEMPO]=700;
    check(legacyStore.save(upgraded,8) && legacyStore.load(read,revision) && revision==8 && read.values[MP_COLUMN_RETURN_TEMPO]==700,
      "Neue vollstaendige Revision speichert den zusaetzlichen Wert neben gueltiger alter Kopie");
    legacyStorage.slots[1].checksum^=1;
    check(legacyStore.load(read,revision) && revision==7 && read.values[MP_TRAVEL_POSITION_ENTER_DEG]==350,
      "Beschaedigte neue Kopie faellt auf die gueltige alte Konfiguration zurueck");
    old.count=84;
    check(!legacyStore.load(read,revision),"Unbekannter alter Parameterumfang wird nicht geraten");
  }
  check(store.save(defaults,1) && store.load(read,revision) && revision==1,
    "Vollstaendige Konfiguration mit Schema und Revision speicher- und lesbar");
  MotionSettings changed=defaults; changed.set(MP_TRAVEL_POSITION_ENTER_DEG,350);
  check(store.save(changed,2) && store.load(read,revision) && revision==2 && read.values[MP_TRAVEL_POSITION_ENTER_DEG]==350,
    "Neueste gueltige Revision stellt geaenderte Fehlertoleranz wieder her");
  storage.tornWrite=true; changed.set(MP_TRAVEL_POSITION_ENTER_DEG,360);
  check(!store.save(changed,3) && store.load(read,revision) && revision==2 && read.values[MP_TRAVEL_POSITION_ENTER_DEG]==350,
    "Unterbrochener Speichervorgang behaelt die vorherige vollstaendige Konfiguration");
  storage.tornWrite=false; storage.failWrite=true;
  check(!store.save(defaults,3) && store.load(read,revision) && revision==2,
    "Speicherfehler wird gemeldet statt neue Werte als gespeichert auszugeben");
  check(!store.save(invalid,3),"Ungueltige Zusammenhaenge gelangen nicht in den Speicher");

  MotionController motion;
  motion.configureLimits(-180,180,-45,30);
  for(uint32_t now=0;now<200;now+=20) motion.tick(now,0,0,true,false);
  MotionSettings tuning=defaults; tuning.set(MP_TRAVEL_START_TIMEOUT_MS,500);
  check(motion.configure(tuning),"Benutzerkonfiguration bei bestaetigtem Stillstand uebernommen");
  check(motion.command(0,2,500,500),"Konfigurierte Fahrt angenommen");
  check(!motion.configure(defaults) && motion.settings().values[MP_TRAVEL_START_TIMEOUT_MS]==500,
    "Laufende Fahrt verhindert Konfigurationswechsel und behaelt bisherige Werte");
  for(uint32_t now=200;now<=800 && !motion.fault();now+=20) motion.tick(now,0,0,true,true);
  check(motion.fault()!=0 && motion.status(true).signedPwm==0,
    "Geaenderte Anlaufzeit wirkt im echten Regler statt nur im Parameter-Ruecklesewert");
  motion.disable();
  check(!motion.configure(invalid),"Ungueltige Gesamtkonfiguration auch am Regler abgelehnt");
  return checks;
}
