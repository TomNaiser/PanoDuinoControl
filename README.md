# PanoDuinoControl

Firmware für einen motorisierten Pan-/Tilt-Panoramakopf mit Encoder-Rückführung und elektrischem Kameraauslöser. Aktueller Quellstand: **2.4.76 / 26. Oktober 2026**, Drahtprotokoll V2. Aktuelles Prüfziel ist der normale **Arduino-Nano-R4-Bluetooth-Build**.

**Quellstand und geprüfter Gerätestand:** 2.4.76 enthält den separaten Versuch zur robusten Tilt-Sensorreferenz. Die am 27. Oktober mit Android 0.6.27 und Windows 27.5.4 bestätigte Produktionsreferenz bleibt die reguläre Firmware **2.4.75 mit Motorprofil Revision 40**; 2.4.76 ist dort nicht installiert. [Aktueller Prüfstand und Pan-Korrektur](https://github.com/TomNaiser/PanoramaSoftware/blob/main/docs/validation/2026-10-27-pan-positioning.md).

## Einstieg

Aktiver Sketch: [PanoDuinoControl/PanoDuinoControl.ino](PanoDuinoControl/PanoDuinoControl.ino). Die Datei `PanoDuinoControlV181231.ino` im Wurzelverzeichnis ist ein historischer Stand.

| Modul | Aufgabe |
|---|---|
| `V2Controller` / `ProtocolV2` | Sitzung, Kommandos, CRC/ACK und Gerätemeldungen |
| `PanoramaExecutor` / `PanoramaPlanV2` | Autonomes Raster-/Bandrezept und Aufnahmephasen |
| `MotionController` | Pan-/Tiltregelung, Ankunft, passive Stabilität und begrenzte Wiederholung |
| `SafetyController` | Grenzen, Referenz und begrenzte Motorfreigabe |
| `MotionSettings` / `MotionConfigStore` | 129 Festkommaparameter, geprüfte Konfiguration und zwei EEPROM-Slots |

Kleine Zielverfehlungen werden achsweise behandelt; die andere Achse beendet ihre zulässige Fahrt. Ausgewählte Bewegungsfehler erhalten einen begrenzten Retry, dann eine erhaltene Pause. Sensor-, Grenz- und Not-Stop-Schutz bleiben vorrangig. Eine reale Fehlerquote von höchstens 1/1.000 ist weiterhin nicht nachgewiesen.

## Bauen und prüfen

```powershell
pwsh -File Tests/build-r4.ps1
```

Benötigt Arduino CLI, Boardpaket `arduino:renesas_uno` 1.6.0 im konfigurierten Toolordner und die [AS5048B-Sensorbibliothek](https://github.com/sosandroid/AMS_AS5048B). Das Skript prüft zusätzlich Speicherreserve und Boot-Stack. Build und Upload sind getrennt; `Tests/upload-r4.ps1` ist kein hardwarefreier Test.

C++-Hosttests: [Tests/FirmwareExecutorTests/FirmwareExecutorTests.vcxproj](Tests/FirmwareExecutorTests/FirmwareExecutorTests.vcxproj), Visual-C++-Toolset v143. Der dokumentierte 2.4.76-Stand bestand 20.295 Assertions mit FakeArduino/Wire. Die [Tilt-Referenzkorrektur](docs/history/TILT-REFERENCE-2.4.76.md) ist zusätzlich mit realem Sensor-Nullregister 0 nach vollständigem Neustart geprüft. USB-Control und Trace sind gesonderte Diagnose-Builds und keine automatische Feldfreigabe.

## Schnittstellen und Clients

- [Gehaltene Fahrt / Jog-Protokoll](PROTOCOL-MANUAL-JOG.md)
- [Manueller Betriebsstatus](docs/history/MANUAL-STATUS-2026-10-12.md)
- [Windows-Client](https://github.com/TomNaiser/PanoramaControl)

Das Gesamtprojekt bietet eine [englische Übersicht](https://github.com/TomNaiser/PanoramaSoftware/blob/main/README.md), ein [kurzes englisches Manual](https://github.com/TomNaiser/PanoramaSoftware/blob/main/docs/en/manual.md) und das [ausführliche deutsche Handbuch](https://github.com/TomNaiser/PanoramaSoftware/blob/main/docs/README.md). Die Clientarchive sind langfristig; die Firmware besitzt kein vollständiges stromausfallsicheres Capture-Archiv. Kamerapulse beweisen keine gespeicherten Fotos. Lokale Toolchains und Firmware-Binaries gehören nicht in den Quellbaum der Veröffentlichung.

Lizenz: [LGPL 2.1](LICENSE).
