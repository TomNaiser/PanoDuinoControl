#ifndef TARGET_ARRIVAL_MONITOR_H
#define TARGET_ARRIVAL_MONITOR_H

#include <stdint.h>

// Bestaetigt ein neues Bewegungsziel erst nach mehreren aufeinanderfolgenden
// Encoder-Samples innerhalb der Regeltoleranzen. Ein Reset ist ausdruecklich
// "nicht erreicht", bis frische Messwerte verarbeitet wurden.
class TargetArrivalMonitor
{
public:
  explicit TargetArrivalMonitor(uint8_t requiredStableSamples)
    : _requiredStableSamples(requiredStableSamples == 0 ? 1 : requiredStableSamples),
      _stableSamples(0), _sampleSequence(0), _panWithinTolerance(false),
      _tiltWithinTolerance(false), _targetActive(false), _reached(false),
      _requirePanProgress(false), _requireTiltProgress(false),
      _panProgressSeen(false), _tiltProgressSeen(false)
  {
  }

  void resetForNewTarget(bool requirePanProgress = false, bool requireTiltProgress = false)
  {
    _stableSamples = 0;
    _panWithinTolerance = false;
    _tiltWithinTolerance = false;
    _targetActive = true;
    _reached = false;
    _requirePanProgress = requirePanProgress;
    _requireTiltProgress = requireTiltProgress;
    _panProgressSeen = false;
    _tiltProgressSeen = false;
  }

  void stop()
  {
    _stableSamples = 0;
    _targetActive = false;
    _reached = false;
  }

  void observe(bool panWithinTolerance, bool tiltWithinTolerance,
               bool panProgress = false, bool tiltProgress = false)
  {
    _sampleSequence++;
    if (panProgress) _panProgressSeen = true;
    if (tiltProgress) _tiltProgressSeen = true;
    _panWithinTolerance = panWithinTolerance;
    _tiltWithinTolerance = tiltWithinTolerance;
    if (!_targetActive || !panWithinTolerance || !tiltWithinTolerance ||
        (_requirePanProgress && !_panProgressSeen) ||
        (_requireTiltProgress && !_tiltProgressSeen))
    {
      _stableSamples = 0;
      _reached = false;
      return;
    }

    if (_stableSamples < _requiredStableSamples) _stableSamples++;
    _reached = _stableSamples >= _requiredStableSamples;
  }

  bool reached() const { return _targetActive && _reached; }
  bool panWithinTolerance() const { return _panWithinTolerance; }
  bool tiltWithinTolerance() const { return _tiltWithinTolerance; }
  uint8_t stableSamples() const { return _stableSamples; }
  uint8_t requiredStableSamples() const { return _requiredStableSamples; }
  uint32_t sampleSequence() const { return _sampleSequence; }

private:
  uint8_t _requiredStableSamples;
  uint8_t _stableSamples;
  uint32_t _sampleSequence;
  bool _panWithinTolerance;
  bool _tiltWithinTolerance;
  bool _targetActive;
  bool _reached;
  bool _requirePanProgress;
  bool _requireTiltProgress;
  bool _panProgressSeen;
  bool _tiltProgressSeen;
};

#endif
