#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace vexlib {
namespace control {

struct PIDGains {
  double kp{0.0}, ki{0.0}, kd{0.0};
  // Feedforward placeholders (enabled in Phase 2)
  double kS{0.0}, kV{0.0}, kA{0.0};
};

struct PIDLimits {
  double out_min{-1.0};
  double out_max{1.0};
  double i_min{-1e9};
  double i_max{ 1e9};
  // Max absolute output change per second (units/s). 0 = disabled.
  double slew_per_s{0.0};
};

struct PIDSettle {
  // Considered in-band if |error| <= err_band AND |error_deriv| <= vel_band
  double err_band{0.0};
  double vel_band{0.0};
  // Milliseconds we must remain in-band to be "settled"
  int dwell_ms{0};
  // Absolute timeout in ms (0 = disabled)
  int timeout_ms{0};
};

enum class AntiWindupMode {
  Clamp,           // clamp integral to [i_min, i_max]
  BackCalculation  // I_aw += beta * (u_sat - u_raw)
};

struct PIDConfig {
  PIDGains gains{};
  PIDLimits limits{};
  PIDSettle settle{};
  AntiWindupMode anti_windup{AntiWindupMode::Clamp};
  // Derivative low-pass filter alpha in [0,1]; 0=no update, 1=no filtering
  double d_alpha{0.2};
  // Optional integral leak per second (0 = disabled). Effective: I *= exp(-leak*dt)
  double i_leak_per_s{0.0};
  // Optional measurement sign (+1 or -1) if sensor direction is inverted
  double meas_sign{+1.0};
  // Optional target sign (+1 or -1) if command convention is inverted
  double out_sign{+1.0};
};

class PID {
public:
  explicit PID(const PIDConfig& cfg);

  // Set a new setpoint; optionally provide a feedforward hint term (Phase 2)
  void setTarget(double setpoint, double feedforward_hint = 0.0);
  double getTarget() const { return target_; }

  // Reset internal state. Optionally provide latest measurement to zero d term.
  void reset(double measured = 0.0);

  // Compute the control output given a new measurement and dt (seconds).
  // Returns saturated output in [out_min, out_max] (and slew-limited if enabled).
  double update(double measured, double dt_s);

  // Status/telemetry
  bool isSettled() const { return settled_; }
  bool isTimedOut() const { return timed_out_; }
  bool isDone() const { return settled_ || timed_out_; }

  double lastOutput() const { return last_out_; }
  double lastError() const { return last_err_; }
  double lastErrorDeriv() const { return last_err_d_filt_; }
  double lastIntegral() const { return integ_; }
  int64_t elapsedMs() const { return elapsed_ms_; }
  int64_t inBandMs() const { return inband_ms_; }

  // Live tuning
  void setGains(const PIDGains& g) { cfg_.gains = g; }
  void setLimits(const PIDLimits& l) { cfg_.limits = l; }
  void setSettle(const PIDSettle& s) { cfg_.settle = s; }
  void setAntiWindup(AntiWindupMode m) { cfg_.anti_windup = m; }
  void setDerivativeAlpha(double a) { cfg_.d_alpha = std::clamp(a, 0.0, 1.0); }
  void setIntegralLeak(double leak_per_s) { cfg_.i_leak_per_s = std::max(0.0, leak_per_s); }
  void setSigns(double meas_sign, double out_sign) { cfg_.meas_sign = meas_sign; cfg_.out_sign = out_sign; }

private:
  PIDConfig cfg_;

  // State
  double target_{0.0};
  double ff_hint_{0.0};        // Phase 2 hook
  double last_meas_{0.0};
  double last_err_{0.0};
  double last_err_d_raw_{0.0};
  double last_err_d_filt_{0.0};
  double integ_{0.0};
  double last_out_{0.0};

  // Status timers
  int64_t elapsed_ms_{0};
  int64_t inband_ms_{0};
  bool settled_{false};
  bool timed_out_{false};

  // Helpers
  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }
  double applySlew(double desired, double dt_s) const;
  void updateBandsAndTimers(double err, double err_d, double dt_s);
};

} // namespace control
} // namespace vexlib
