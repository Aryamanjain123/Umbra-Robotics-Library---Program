#pragma once
#include <cmath>
#include <vector>
#include <algorithm>

namespace vexlib {
namespace motion {

struct ProfileConstraints {
  double max_vel{0.0};   // max velocity
  double max_acc{0.0};   // max acceleration
  double max_jerk{0.0};  // max jerk (for S-curve only, 0 disables jerk limiting)
};

struct ProfileState {
  double pos{0.0};
  double vel{0.0};
  double acc{0.0};
  double t{0.0};
};

class MotionProfile {
public:
  virtual ~MotionProfile() = default;
  virtual ProfileState sample(double t) const = 0;
  virtual double duration() const = 0;
};

// ---------------- Trapezoidal Profile ----------------
class TrapezoidProfile : public MotionProfile {
public:
  TrapezoidProfile(double start, double goal, const ProfileConstraints& c);

  ProfileState sample(double t) const override;
  double duration() const override { return t_total_; }

private:
  double x0_, xf_;
  double dir_;   // +1 or -1
  ProfileConstraints c_;
  double t_acc_, t_cruise_, t_dec_, t_total_;
  double v_peak_;
};

// ---------------- S-Curve Profile ----------------
class SCurveProfile : public MotionProfile {
public:
  SCurveProfile(double start, double goal, const ProfileConstraints& c);

  ProfileState sample(double t) const override;
  double duration() const override { return t_total_; }

private:
  double x0_, xf_;
  double dir_;
  ProfileConstraints c_;
  double t_j_, t_a_, t_v_, t_total_;
};

} // namespace motion
} // namespace vexlib
