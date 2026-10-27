# Tilt-Sensorreferenz in 2.4.76

## Fehler und Korrektur

2.4.75 schreibt beim Arduino-Start einmal `zeroRegW(2396)` in den AS5048B.
Das Register ist flüchtig. Nach einem Sensorneustart ohne entsprechende
Neureferenzierung wird der ausgelesene Winkel um 2396/16384 Umdrehung, also
52,646484375°, verschoben. Benutzer-Nullpunkt und mechanische Lage ändern
sich dabei nicht. Ein unvollständiger Startschreibzugriff ist ebenfalls
nicht abgesichert.

2.4.76 schreibt diesen Sensor-Nullpunkt nicht mehr. Der Tilt-Leser erfasst
stattdessen das aktuelle Nullregister vor und nach dem Winkel und normiert
die Probe auf dieselbe feste mechanische Referenz 2396:

```text
normalized = (angle + observedZero + 16384 - 2396) & 0x3fff
```

Unterschiedliche Referenzen vor/nach dem Winkel ergeben eine ungültige Probe.
Unvollständige Übertragungen bleiben Sensorfehler. Der bisherige gemeinsame
8-ms-Zeitbedarf für Encoderzugriffe und das 2-ms-Transferlimit bleiben wirksam.
Tilt benötigt jetzt zwei zusätzliche Registerlesezugriffe pro Probe.
Pan-Leser, Benutzer-Nullpunkt, Grenzen, Motorparameter und `MotionController`
werden nicht verändert. Der Trace-Build gibt im Stillstand das beobachtete
Nullregister auf USB als `[ENCODER_REFERENCE]` aus.

## Nachweise am 26./27. Oktober 2026

- Der Regressionstest `testTiltSensorReferenceSurvivesSensorReset` erreicht
  die echte Firmware-Lesefunktion. Er reproduziert vor der Korrektur den
  52,6°-Sprung. Danach bestehen neun Prüfungen, einschließlich verschiedener
  Nullregisterwerte, Referenzwechsel während der Probe und verkürzter Antwort.
- Vollständige Firmware-Hostsuite: 20.295 Assertions bestanden. Regulärer
  Nano-R4- und separater USB-Trace-Build bestehen; Konstruktor-Startstack
  240 Byte bei einem Budget von 384 Byte.
- Eine instrumentierte Original-2.4.75 erfasst im Stillstand den Sprung von
  −7,581° auf −60,271°. Alle 4.582 Proben der beiden Teilaufzeichnungen haben
  Motorfreigabe aus und beide PWM null, keine verworfenen Frames. Erst 2,565 s
  nach dem Winkelsprung folgt der erste registrierte I²C-Fehler. Der Busclear
  hat diesen konkreten Sprung deshalb nicht verursacht. Das damalige reale
  Sensor-Nullregister enthält diese alte Spur noch nicht.
- Mit 2.4.76 wird vor dem Versorgungsneustart tatsächlich Nullregister **2396**
  gelesen. Nach vollständigem Trennen von USB und Hauptversorgung wird
  tatsächlich **0** gelesen, während Tilt bei ungefähr **−7,6°** bleibt.
  Der Neustarttest enthält 1.354 vollständige Proben über 29,935 s, ohne
  Sensorfehler und ohne Motoransteuerung. Größte Abtastlücke: 25 ms.
- Alle 129 Motorparameter bleiben identisch, Konfigurationsrevision 33.

Die alleinige Minigorilla-Unterbrechung hatte das Sensorregister bei diesem
Aufbau noch nicht zurückgesetzt (weiter 2396). Sie wird nicht als Nachweis
eines Sensorneustarts bei laufendem Arduino gezählt. Der reale vollständige
Neustart prüft die Normierung bei Nullregister 0; Referenzwechsel im laufenden
Lesepfad werden zusätzlich durch die gezielten Hosttests geprüft.

## Aussagegrenzen

Die elektrische Ursache eines Sensorneustarts bzw. der aufgezeichneten
I²C-Aussetzer ist damit nicht geklärt. Kommunikationsfehler bleiben ein
Stopgrund. Diese Änderung korrigiert die Abhängigkeit vom flüchtigen
Referenzregister; sie ist kein nachgewiesener Fix des ursprünglich gemeldeten
Pan-Überschwingens oder kleiner Tilt-Zielverfehlungen bei Aufnahmefahrten.

Rohdaten und reproduzierbare Parser liegen im übergeordneten Arbeitsbereich
unter `analysis/2026-10-26-surface`, insbesondere `ANDROID-AUSWERTUNG-20261027.md`,
`reference2476-validation-20261027.json` und den `usb-reference2476-*.log`-
Aufzeichnungen. Binaries und Rohlogs gehören nicht in die Veröffentlichung.
