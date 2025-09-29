#include "vexlib/control/pid.hpp"
#include <cmath>

namespace vexlib {
namespace control {

PID::PID(const PIDConfig& cfg) : cfg_(cfg) { reset(0.0); }

void PID::setTarget(double setpoint) {
  target_ = setpoint;
}

void PID::setFFState(double desired_vel, double desired_accel) {
  ff_vel_  = desired_vel;
  ff_acc_  = desired_accel;
}

void PID::reset(double measured) {
  last_meas_ = measured * cfg_.meas_sign;
  const double err = (target_ - last_meas_);
  last_err_ = err;
  last_err_d_raw_ = 0.0;
  last_err_d_filt_ = 0.0;
  integ_ = 0.0;
  last_out_ = 0.0;
  last_ff_ = 0.0;
  elapsed_ms_ = 0;
  inband_ms_ = 0;
  settled_ = false;
  timed_out_ = false;
}

double PID::applySlew(double desired, double dt_s) const {
  if (cfg_.limits.slew_per_s <= 0.0 || dt_s <= 0.0) return desired;
  const double max_delta = cfg_.limits.slew_per_s * dt_s;
  const double delta = desired - last_out_;
  if (delta >  max_delta) return last_out_ + max_delta;
  if (delta < -max_delta) return last_out_ - max_delta;
  return desired;
}

void PID::updateBandsAndTimers(double err, double err_d, double dt_s) {
  if (dt_s <= 0.0) return;

  elapsed_ms_ += static_cast<int64_t>(dt_s * 1000.0);

  const bool in_err = std::fabs(err) <= cfg_.settle.err_band;
  const bool in_vel = std::fabs(err_d) <= cfg_.settle.vel_band;

  if (in_err && in_vel) {
    inband_ms_ += static_cast<int64_t>(dt_s * 1000.0);
  } else {
    inband_ms_ = 0;
  }

  if (cfg_.settle.dwell_ms > 0 && inband_ms_ >= cfg_.settle.dwell_ms) {
    settled_ = true;
  }

  if (cfg_.settle.timeout_ms > 0 && elapsed_ms_ >= cfg_.settle.timeout_ms) {
    timed_out_ = true;
  }
}

double PID::update(double measured, double dt_s) {
  // Direction/sign handling
  const double meas = measured * cfg_.meas_sign;
  const double err = (target_ - meas);

  // Derivative (raw)
  double err_d = 0.0;
  if (dt_s > 0.0) {
    err_d = (err - last_err_) / dt_s;
  }

  // Low-pass filter derivative on error
  const double a = std::clamp(cfg_.d_alpha, 0.0, 1.0);
  last_err_d_filt_ = a * err_d + (1.0 - a) * last_err_d_filt_;
  last_err_d_raw_ = err_d;

  // Integral with optional exponential leak
  if (cfg_.i_leak_per_s > 0.0 && dt_s > 0.0) {
    const double decay = std::exp(-cfg_.i_leak_per_s * dt_s);
    integ_ *= decay;
  }
  if (dt_s > 0.0) {
    integ_ += err * dt_s;
  }

  // Compute PID (no FF yet)
  const double P = cfg_.gains.kp * err;
  const double I = cfg_.gains.ki * integ_;
  const double D = cfg_.gains.kd * last_err_d_filt_;
  double u_pid = (P + I + D);

  // Feedforward contribution using desired kinematics
  double u_ff = 0.0;
  if (cfg_.ff_enabled) {
    const double s = sign_smooth(ff_vel_, cfg_.ff_sign_epsilon);
    u_ff = cfg_.gains.kS * s + cfg_.gains.kV * ff_vel_ + cfg_.gains.kA * ff_acc_;
  }
  last_ff_ = u_ff;

  // Combine, apply output sign, then saturate
  double u_raw = (u_pid + u_ff) * cfg_.out_sign;

  const double u_sat = clamp(u_raw, cfg_.limits.out_min, cfg_.limits.out_max);

  // Anti-windup
  if (cfg_.anti_windup == AntiWindupMode::Clamp) {
    integ_ = clamp(integ_, cfg_.limits.i_min, cfg_.limits.i_max);
  } else {
    const double beta = 1.0; // could be exposed if needed
    const double eps = 1e-9;
    const double ki = std::max(std::fabs(cfg_.gains.ki), eps);
    const double correction = beta * (u_sat - u_raw) / ki;
    integ_ += correction;
    integ_ = clamp(integ_, cfg_.limits.i_min, cfg_.limits.i_max);
  }

  // Slew-rate limiting
  const double u_slew = applySlew(u_sat, dt_s);

  // Timers / settle logic
  updateBandsAndTimers(err, last_err_d_filt_, dt_s);

  // Commit state
  last_meas_ = meas;
  last_err_ = err;
  last_out_ = u_slew;

  return last_out_;
}

} // namespace control
} // namespace vexlib
