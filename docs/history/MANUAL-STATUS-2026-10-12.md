# Manueller Betriebsstatus, Firmware 2.4.41

Erweiterung des V2-Protokolls. Die bisherigen STATUS-Felder bleiben unverändert. Alte Clients dürfen die zusätzlichen Nachrichten ignorieren.

`MANUAL_STATUS_CAPABILITIES|1` wird mit den Gerätefähigkeiten übertragen. Erst nach dieser ausdrücklichen Freigabe darf ein Client seinen bisherigen Diagnose-Abschlussweg ersetzen.

`MANUAL_STATUS|commandSequence|kind|result|targetPanMdeg|targetTiltMdeg|durationMs|maximumMs`

| Feld | Bedeutung |
|---|---|
| commandSequence | Sequenz des zuletzt angenommenen MANUAL_MOVE oder TRIGGER_ONCE; 0 vor der ersten Aktion |
| kind | NONE, MOVE oder TRIGGER |
| result | IDLE vor der ersten Aktion; ACTIVE während der Aktion; REACHED für Zielankunft; TRIGGERED für abgeschlossenen Kamera-Puls; CANCELLED, TIMED_OUT oder FAULT für unterbrochene Aktionen |
| targetPanMdeg / targetTiltMdeg | Angefordertes Fahrziel; bei TRIGGER Position zu Beginn des Pulses |
| durationMs | Während ACTIVE verstrichene Zeit; danach beim Abschluss festgehaltene Dauer. Differenz mit uint32-Umlaufbehandlung |
| maximumMs | Für diese Aktion geltende maximale Dauer, einschließlich vorhandener inkrementeller Fahrtgrenze |

NONE/IDLE hat Sequenz, Dauer und Maximaldauer 0. REACHED und TIMED_OUT gehören zu MOVE, TRIGGERED zu TRIGGER. Die anderen Ergebnisse können beide Arten betreffen.

Der Controller sendet die Nachricht nach Annahme einer manuellen Aktion und zusammen mit jeder STATUS-Meldung, also auch beim Abschluss und bei GET_STATUS. Das letzte Ergebnis bleibt bis zur nächsten angenommenen manuellen Aktion erhalten. NACK und gecachter Retry verändern es nicht. Ein neuer Firmware-Boot beginnt mit NONE/IDLE; die bekannte DEVICE_SESSION-Identität grenzt die Lebensdauer ab.

Die Clients müssen das Ergebnis anhand der angenommenen Befehlssequenz und Aktionsart zuordnen und bei MOVE das Ziel vergleichen. Eine vor der zugehörigen ACK empfangene Statusmeldung darf keinen Abschluss bestätigen: uint16-Sequenzen werden wiederverwendet. Nach der ACK folgt nötigenfalls die nächste periodische Wiederholung des gespeicherten Ergebnisses. Bei Verbindungs-/Bootwechsel werden alte lokale Aktionsbeobachter verworfen und keine alte Bewegung wiederholt.

REACHED beruht auf der vorhandenen hardwareseitigen Ankunftserkennung. Der bisherige ruhige Haltezustand wird nicht verändert. REACHED bedeutet deshalb keine ausdrückliche Bestätigung von PWM=0. TRIGGERED bestätigt den abgeschlossenen Geräte-Puls, keine Datei in der Kamera. Aufnahmeereignisse und deren Speicherung bleiben eigenständig.
