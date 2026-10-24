# PanoDuinoControl

Stand 23. Oktober 2026: Normale R4-Bluetooth-Firmware **2.4.75** installiert. Eine ruhende Achse außerhalb der Zieltoleranz schaltet nur ihren eigenen Ausgang ab; die andere Achse beendet ihre begrenzte Fahrt vor der Qualitätswarnung. Der zusätzlich real beobachtete Ruhefehler `MOTION_NOT_QUIET` erhält jetzt dieselbe einmalige automatische Wiederholung wie ähnliche Bewegungsfehler; anhaltende Probleme pausieren mit erhaltenem Auftrag. Die strenge Ruheprüfung bleibt unverändert, Sensorfehler und Not-Stop behalten Vorrang. **20.286 Assertions** bestanden. Basislauf und getrennte Prüfung der Korrekturen stehen im [Bericht des Gesamtprojekts](../analysis/2026-10-23-acquisition/ERGEBNIS.md). Eine reale 1/1000-Ausfallrate der neuen Version ist noch nicht nachgewiesen.

Vorheriger Stand 22. Oktober 2026: Normale R4-Bluetooth-Firmware **2.4.73**. Ein einzelner behebbarer Bewegungsfehler wird ohne Benutzer-RESUME einmal am selben Bildpunkt wiederholt; Zweitfehler pausieren mit erhaltenem Auftrag. ESTOP und Sensor-/Grenzschutz bleiben wirksam. **17.288 Assertions** bestanden, einschließlich 4000 Bildern mit 42 injizierten Stalls und vier weiteren 1000-Bilder-Serien. Dies belegt die Fortsetzungslogik, keine reale 1/1000-Ausfallrate. Reale Messungen und 20-%-Überlappungsbewertung im [Bericht des Gesamtprojekts](../analysis/2026-10-22-robustness/ERGEBNIS.md).

Arduino  based control software for an automated pan/tilt platform for Gigapixel Panorama Acquisition

Features:

-wireless communication with PanoramaControl front end

-PID closed loop position control of  modified RC-Servos (operated via external H-Bridge)

-high positional accuracy by using 14-bit rotary encoder data

-remote triggering of camera shutters 


Requirements:  

-PanoramaControl software (which can also be found in my Github repository: https://github.com/TomNaiser/PanoramaControl)

-Arduino Lib for AMS AS5048B I2C - 14-bit magnetic rotary position sensor 
https://github.com/sosandroid/AMS_AS5048B   (under the BSD license)







Search tags: arduino gigapixel panorama pan tilt motorized head controller software PID servo encoder
