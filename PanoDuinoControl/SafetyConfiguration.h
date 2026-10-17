#ifndef SAFETY_CONFIGURATION_H
#define SAFETY_CONFIGURATION_H

// Tilt-Endpositionen wurden am realen Aufbau vermessen; fuer die erste
// motorisierte Erprobung gelten die unten eingetragenen engeren Grenzen.
// Nur mit beaufsichtigter, langsamer Fahrt und erreichbarer Stromabschaltung.
// Bei 0 verweigert Protokoll V2 sämtliche Fahrbefehle und autonomen Pläne.
#ifndef SAFETY_LIMITS_CONFIRMED
#define SAFETY_LIMITS_CONFIRMED 1
#endif

// Pan folgt den bisherigen PC-Grenzen (C:\PanoControlConfig.xml, 27.09.2026).
// Die bisher am Aufbau verwendeten Encodergrenzen bleiben unveraendert.
// Seit Android 0.6.2 bedeutet zunehmender Tiltwinkel: Objektiv nach oben;
// das Diagramm stellt positive Winkel oben dar. Ein benutzerdefinierter
// Nullpunkt verschiebt nur die numerischen Tiltwerte. Die hier definierten
// Rohgrenzen behalten dieselben physischen Endlagen. Pan hat laut Nutzer
// keine mechanische Begrenzung; Ziele bleiben auf -180..+180 begrenzt.
#ifndef PAN_HARD_MIN_MDEG
#define PAN_HARD_MIN_MDEG  (-180000L)
#endif
#ifndef PAN_HARD_MAX_MDEG
#define PAN_HARD_MAX_MDEG  ( 180000L)
#endif
#ifndef TILT_HARD_MIN_MDEG
#define TILT_HARD_MIN_MDEG ( -45000L)
#endif
#ifndef TILT_HARD_MAX_MDEG
#define TILT_HARD_MAX_MDEG (  50000L)
#endif
// Aeußerste, am realen Aufbau gemessene Endlagen ohne Reserve. Innerhalb
// dieses Bereichs kann die GUI die wirksamen Grenzen dauerhaft einstellen.
// Diese Werte beziehen sich immer auf die unveraenderte Encoder-Werkreferenz.
#define TILT_ABSOLUTE_MIN_MDEG (-50000L)
#define TILT_ABSOLUTE_MAX_MDEG ( 55000L)

#endif
