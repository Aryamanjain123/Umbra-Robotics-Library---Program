#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "vexlib/hal/motor.hpp"

namespace vexlib::drive {

// Simple 2-motor tank / 4-motor tank via motor groups
class TankChassis {
 public:
  // Provide left/right motors (they can be plugged into a motor group externally if desired)
  TankChassis(hal::Motor& left, hal::Motor& right, double max_volts = 12.0)
      : left_(left), right_(right), maxV_(max_volts) {}

  // Arcade drive (fwd [-1..1], turn [-1..1])
  void arcade(double fwd, double turn) {
    double l = std::clamp(fwd + turn, -1.0, 1.0);
    double r = std::clamp(fwd - turn, -1.0, 1.0);
    left_.set_voltage(l * maxV_);
    right_.set_voltage(r * maxV_);
  }

  // Curvature drive (CheesyDrive-ish)
  void curvature(double fwd, double curvature, bool allow_quickturn) {
    double angular = allow_quickturn ? curvature : std::abs(fwd) * curvature;
    arcade(fwd, angular);
  }

  void brake(hal::BrakeMode mode) {
    left_.set_brake(mode);
    right_.set_brake(mode);
  }

 private:
  hal::Motor& left_;
  hal::Motor& right_;
  double maxV_;
};

}  // namespace vexlib::drive
