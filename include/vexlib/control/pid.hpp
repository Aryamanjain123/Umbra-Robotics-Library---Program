#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include "vexlib/units.hpp"

namespace vexlib::control {

class PID {
 public:
  PID(double kP, double kI, double kD, double max_out = 12.0)
      : kP_(kP), kI_(kI), kD_(kD), maxOut_(max_out) {}

  void reset() { 
    integ_ = 0.0; 
    prevErr_ = std::nullopt; 
  }

  double update(double error, double dt) {
    double p = kP_ * error;
    if (prevErr_.has_value()) {
      integ_ += error * dt;
      // Anti-windup clamp
      if (kI_ > 0.0) {
        integ_ = std::clamp(integ_, -maxOut_ / kI_, maxOut_ / kI_);
      }
      deriv_ = (error - *prevErr_) / dt;
    } else {
      deriv_ = 0.0;
    }
    prevErr_ = error;
    double out = p + kI_ * integ_ + kD_ * deriv_;
    return std::clamp(out, -maxOut_, maxOut_);
  }

  void set_gains(double kp, double ki, double kd) { 
    kP_ = kp; 
    kI_ = ki; 
    kD_ = kd; 
  }

 private:
  double kP_, kI_, kD_;
  double maxOut_;
  double integ_{0.0};
  double deriv_{0.0};
  std::optional<double> prevErr_;
};

}  // namespace vexlib::control
