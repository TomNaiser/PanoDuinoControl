#ifndef PANODUINO_MOTION_CONFIGURATION_H
#define PANODUINO_MOTION_CONFIGURATION_H

// Historical arrival-monitor constants, retained for the legacy monitor tests.
// The R4 MotionController uses the independent MOTION_* configuration below.
#define TARGET_CURRENT_ERROR_MAX_DEG 1.5
#define TARGET_HISTORY_SQUARED_ERROR_MAX_DEG2 10.0
#define TARGET_STABLE_SAMPLES 5
// A one-degree manual step starts inside the 1.5-degree arrival tolerance.
// Require actual encoder progress before declaring such a move complete.
#define TARGET_PROGRESS_REQUIRED_FROM_DEG 0.75
#define TARGET_PROGRESS_MIN_DEG 0.20

// Maximale Zeit fuer eine komplette Fahrt, einschliesslich Rueckkehr aus einer
// instabilen Beruhigungsphase. Der Timer beginnt beim Setzen des Ziels und wird
// durch Nachschwingen nicht neu gestartet.
#define PANORAMA_MOVEMENT_TIMEOUT_MS 120000UL

// Provisional R4 tuning. Speed is a profile constraint, never a PWM multiplier.
// These values need a supervised acceptance test with the real camera/load.
#define MOTION_CONTROL_PERIOD_MS 20UL
#define MOTION_MAX_SAMPLE_GAP_MS 100UL
#define MOTION_ENCODER_TRANSFER_TIMEOUT_US 2000U
#define MOTION_ENCODER_RECOVERY_BUDGET_US 8000UL
#define MOTION_ENCODER_RECOVERY_INTERVAL_MS 1000UL
#define MOTION_ENCODER_RECOVERY_BURST_MAX 3
#define MOTION_CONFIRM_MS 250UL
#define MOTION_POSITION_ENTER_DEG 0.15
#define MOTION_POSITION_EXIT_DEG 0.25
#define MOTION_VELOCITY_ENTER_DPS 0.25
#define MOTION_VELOCITY_EXIT_DPS 0.50
#define MOTION_QUIET_SPAN_DEG 0.12
#define MOTION_HISTORY_LENGTH 10
#define MOTION_FEEDBACK_HISTORY_LENGTH 5 // Timestamped slope across normally 80 ms.
#define MOTION_REVERSAL_PWM_MAX 40.0
#define MOTION_PWM_SLEW_PER_SECOND 250.0
#define MOTION_PROGRESS_DEG 0.044
#define MOTION_NO_PROGRESS_MS 1500UL
#define MOTION_REVERSAL_TIMEOUT_MS 2500UL
#define MOTION_TRACKING_MAX_DEG 0.50
#define MOTION_TRACKING_INTEGRAL_PWM_MAX 40.0
#define MOTION_STALL_PWM_MIN 15.0
#define MOTION_MANUAL_SPEED_PERMILLE 500
#define MOTION_TILT_BRAKE_LOOKAHEAD_SECONDS 0.06 // provisional unloaded feedback/drive lag
// Experimental unloaded Tilt travel. Running floors come from the range
// measurements; below them the requested speed may not be continuously
// attainable. Arrival accuracy and mechanical quiet remain separate checks.
#define MOTION_TRAVEL_POSITION_ENTER_DEG 0.30
#define MOTION_TRAVEL_POSITION_EXIT_DEG 0.40
#define MOTION_TRAVEL_START_TIMEOUT_MS 1500UL
#define MOTION_TRAVEL_SETTLE_TIMEOUT_MS 5000UL
#define MOTION_TRAVEL_START_PROGRESS_DEG 0.05
// One short reduction on early two-sample progress while full confirmation
// is still pending; false/noisy evidence never extends the startup timeout.
#define MOTION_TRAVEL_START_EARLY_PROGRESS_DEG 0.011
#define MOTION_TRAVEL_START_RELIEF_MS 100UL
#define MOTION_TRAVEL_START_PRELOAD_SPEED_FRACTION 0.7
// Raise torque promptly on slow running feedback; retain the same cap/watchdog.
#define MOTION_TRAVEL_RECOVERY_INTEGRAL_GAIN 40.0
// Preserve learned uphill running torque through brief encoder speed peaks.
// The proportional correction still removes excess output immediately.
#define MOTION_TRAVEL_UP_UNWIND_INTEGRAL_GAIN 8.0
// Immediate uphill correction during weak progress reduces handoff stiction
// without applying the same extra correction throughout normal cruise.
#define MOTION_TRAVEL_UP_RECOVERY_PROPORTIONAL_GAIN 5.0
// Initial loaded handoff may still contain the first breakaway speed peak.
// Use fresh weak-progress feedback only within this bounded first degree.
#define MOTION_TRAVEL_UP_HANDOFF_PROGRESS_DEG 1.0
#define MOTION_TRAVEL_BRAKE_LEAD_SECONDS 0.05
#define MOTION_TRAVEL_START_BRAKE_LEAD_SECONDS 0.075
// Camera downhill coast requires an earlier provisional power-off point.
#define MOTION_TRAVEL_DOWN_BRAKE_LEAD_SECONDS 0.08
#define MOTION_TRAVEL_PWM_MAX 40.0
// Bound downhill breakaway energy before velocity feedback has caught up.
#define MOTION_TRAVEL_DOWN_START_PWM_MAX 34.0
// Small encoder excursions can enter cruise before true downhill breakaway.
// Retain its torque cap until travel clearly exceeds the observed noise.
#define MOTION_TRAVEL_DOWN_START_CONFIRM_PROGRESS_DEG 0.30
// Local unloaded cruise measurements permit reducing the running bias by
// four PWM counts (+15 to +11, -25 to -21); start torque remains separate.
#define MOTION_TRAVEL_PWM_REDUCTION_MAX 4.0
// Camera load sustains about 5 dps at -21; permit further downhill speed
// correction while retaining the higher bias whenever progress weakens.
#define MOTION_TRAVEL_DOWN_PWM_REDUCTION_MAX 10.0

struct AxisMotionConfiguration
{
  double maximumVelocity;      // degrees / second
  double maximumAcceleration;  // degrees / second^2
  double maximumJerk;          // degrees / second^3
  double proportionalGain;    // PWM / degree
  double velocityGain;        // PWM / (degree / second)
  double maximumPwm;          // existing 8-bit PWM scale, not percent
  double frictionFeedforwardPwm; // provisional unloaded drive compensation
  double velocityFeedforwardGain;
  double negativeFrictionFeedforwardPwm; // magnitude for negative travel
  double velocityIntegralGain; // PWM / degree; zero retains the PD controller
  double positionVelocityGain; // degrees/second per degree, for velocity PI
  double positiveRunningPwm; // zero on either side disables staged travel
  double negativeRunningPwm;
  double trackingIntegralGain; // PWM / (degree second), for the profiled PD axis
};

inline AxisMotionConfiguration panMotionConfiguration()
{
  AxisMotionConfiguration value = {8.0, 12.0, 40.0, 40.0, 5.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 40.0};
  return value;
}

inline AxisMotionConfiguration tiltMotionConfiguration()
{
  // Unloaded range measurements: +15 continues slowly, -20 often stalls,
  // -25 continues at 3-5 dps. The previous 5 PWM/dps feedforward and high
  // velocity feedback repeatedly braked the soft drive back into stiction.
  // A bounded velocity PI takes up breakaway torque and load differences;
  // position error adjusts requested speed instead of large direct PWM jumps.
  // These values still require real and loaded acceptance.
  AxisMotionConfiguration value = {5.0, 8.0, 24.0, 45.0, 2.0, 100.0, 14.0, 1.0, 22.0, 12.0, 2.0, 15.0, 25.0, 0.0};
  return value;
}

#endif




