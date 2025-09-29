#include "vexlib/motion/profile.hpp"

namespace vexlib {
namespace motion {

// -------- TrapezoidProfile --------
TrapezoidProfile::TrapezoidProfile(double start, double goal, const ProfileConstraints& c)
  : x0_(start), xf_(goal), c_(c) {
  dir_ = (xf_ >= x0_) ? 1.0 : -1.0;
  double dist = std::fabs(xf_ - x0_);

  // Time to accel to max_vel
  t_acc_ = c_.max_vel / c_.max_acc;
  double d_acc = 0.5 * c_.max_acc * t_acc_ * t_acc_;

  if (2*d_acc > dist) {
    // Triangular profile (never hit max_vel)
    t_acc_ = std::sqrt(dist / c_.max_acc);
    v_peak_ = c_.max_acc * t_acc_;
    t_cruise_ = 0;
    t_dec_ = t_acc_;
    t_total_ = 2 * t_acc_;
  } else {
    // Trapezoidal profile
    v_peak_ = c_.max_vel;
    double d_cruise = dist - 2*d_acc;
    t_cruise_ = d_cruise / c_.max_vel;
    t_dec_ = t_acc_;
    t_total_ = t_acc_ + t_cruise_ + t_dec_;
  }
}

ProfileState TrapezoidProfile::sample(double t) const {
  t = std::clamp(t, 0.0, t_total_);
  ProfileState s; s.t = t;

  if (t < t_acc_) {
    // accel phase
    s.acc = c_.max_acc * dir_;
    s.vel = s.acc * t;
    s.pos = x0_ + 0.5 * s.acc * t * t;
  } else if (t < t_acc_ + t_cruise_) {
    // cruise phase
    s.acc = 0;
    s.vel = v_peak_ * dir_;
    double d_acc = 0.5 * c_.max_acc * t_acc_ * t_acc_;
    double d_cruise = (t - t_acc_) * v_peak_;
    s.pos = x0_ + dir_ * (d_acc + d_cruise);
  } else {
    // decel phase
    double td = t - (t_acc_ + t_cruise_);
    s.acc = -c_.max_acc * dir_;
    s.vel = v_peak_*dir_ + s.acc * td;
    double d_acc = 0.5 * c_.max_acc * t_acc_ * t_acc_;
    double d_cruise = t_cruise_ * v_peak_;
    double d_dec = v_peak_*td - 0.5*c_.max_acc*td*td;
    s.pos = x0_ + dir_ * (d_acc + d_cruise + d_dec);
  }

  return s;
}

// -------- SCurveProfile (jerk-limited S-curve) --------
SCurveProfile::SCurveProfile(double start, double goal, const ProfileConstraints& c)
  : x0_(start), xf_(goal), c_(c) {
  dir_ = (xf_ >= x0_) ? 1.0 : -1.0;
  double dist = std::fabs(xf_ - x0_);

  if (c_.max_jerk <= 0.0) {
    // Fallback to trapezoid
    t_j_ = 0; t_a_ = 0; t_v_ = dist / c_.max_vel;
    t_total_ = t_v_;
  } else {
    // Approximate 7-phase S-curve: jerk, accel, cruise, decel, etc.
    // For simplicity, assume symmetric accel/decel
    t_j_ = c_.max_acc / c_.max_jerk;       // time to ramp accel
    t_a_ = t_j_ + (c_.max_vel / c_.max_acc); // total accel time
    double d_acc = 0.5 * c_.max_acc * t_a_ * t_a_; // rough
    double d_cruise = std::max(0.0, dist - 2*d_acc);
    t_v_ = d_cruise / c_.max_vel;
    t_total_ = 2*t_a_ + t_v_;
  }
}

ProfileState SCurveProfile::sample(double t) const {
  t = std::clamp(t, 0.0, t_total_);
  ProfileState s; s.t = t;

  // Approximate: use trapezoid kinematics but smooth edges by reducing jerk
  // In a full implementation, you'd integrate jerk across 7 phases.
  if (t < t_a_) {
    s.acc = c_.max_acc * dir_ * (t / t_a_); // linearly increasing accel
    s.vel = 0.5 * s.acc * t;
    s.pos = x0_ + 0.5 * s.acc * t * t / 2.0;
  } else if (t < t_a_ + t_v_) {
    s.acc = 0;
    s.vel = c_.max_vel * dir_;
    s.pos = x0_ + dir_ * (c_.max_vel * (t - t_a_));
  } else {
    double td = t - (t_a_ + t_v_);
    s.acc = -c_.max_acc * dir_ * (td / t_a_);
    s.vel = c_.max_vel*dir_ - 0.5*c_.max_acc*td;
    s.pos = xf_ - 0.5 * s.acc * td * td;
  }

  return s;
}

} // namespace motion
} // namespace vexlib
