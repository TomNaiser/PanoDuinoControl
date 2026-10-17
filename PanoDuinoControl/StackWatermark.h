#ifndef PANODUINO_STACK_WATERMARK_H
#define PANODUINO_STACK_WATERMARK_H

#include <Arduino.h>
#include <stdint.h>

// Der ATmega328P besitzt keinen Speicherschutz. Heap und Stack wachsen
// aufeinander zu. Dieser Monitor markiert nach Abschluss der Initialisierung
// den freien Zwischenraum und kann deshalb auch kurze, spaeter wieder
// freigegebene Stackspitzen erkennen.
class StackWatermark
{
public:
  StackWatermark() : _paintedBytes(0) {}

  void begin()
  {
#ifdef __AVR__
    // Der aktuelle Build benutzt keinen Heap. Falls eine spaetere globale
    // Initialisierung bereits allokiert hat, nicht hinter einer unbekannten
    // Heapbelegung markieren.
    extern void *__brkval;
    if (__brkval != 0)
    {
      _paintedBytes = 0;
      return;
    }
    uint8_t *start = heapStart();
    uint16_t stackPointer = SP;
    uint16_t startAddress = (uint16_t)start;
    const uint8_t guardBytes = 16;
    if (stackPointer <= startAddress + guardBytes)
    {
      _paintedBytes = 0;
      return;
    }
    _paintedBytes = (uint16_t)(stackPointer - startAddress - guardBytes);
    for (uint16_t index = 0; index < _paintedBytes; index++) start[index] = WATERMARK;
#else
    _paintedBytes = 0;
#endif
  }

  uint16_t currentFreeBytes() const
  {
#ifdef __AVR__
    uint16_t stackPointer = SP;
    uint16_t heapAddress = (uint16_t)heapEnd();
    return stackPointer > heapAddress ? (uint16_t)(stackPointer - heapAddress) : 0;
#else
    return 0;
#endif
  }

  uint16_t minimumFreeBytes() const
  {
#ifdef __AVR__
    // Immer ab dem urspruenglich markierten Heapanfang scannen. Eine spaetere
    // Heapallokation ueberschreibt dort das Muster und liefert sicher 0,
    // statt den Scanbereich unbemerkt nach oben zu verschieben.
    uint8_t *start = heapStart();
    uint16_t untouched = 0;
    while (untouched < _paintedBytes && start[untouched] == WATERMARK) untouched++;
    // Der beim Markieren ausgesparte Sicherheitsabstand wird absichtlich
    // nicht addiert. Der gemeldete Tiefstwert ist damit konservativ.
    return untouched;
#else
    return 0;
#endif
  }

private:
#ifdef __AVR__
  static const uint8_t WATERMARK = 0xA5;

  static uint8_t *heapStart()
  {
    extern char __heap_start;
    return (uint8_t *)&__heap_start;
  }

  static uint8_t *heapEnd()
  {
    extern void *__brkval;
    return __brkval == 0 ? heapStart() : (uint8_t *)__brkval;
  }
#endif

  uint16_t _paintedBytes;
};

#endif
