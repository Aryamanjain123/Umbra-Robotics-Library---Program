#pragma once
#include <cmath>
#include "vexlib/units.hpp"

namespace vexlib::localization {

struct Pose2d {
  double x_m{0.0}, y_m{0.0}, yaw_rad{0.0};
};

class TwoWheelOdometry {
 public:
  TwoWheelOdometry(double wheel_circum_m, double trackwidth_m)
      : wc_(wheel_circum_m), tw_(trackwidth_m) {}

  Pose2d update(double left_deg, double right_deg, double yaw_deg) {
    double l_m = (left_deg / 360.0) * wc_;
    double r_m = (right_deg / 360.0) * wc_;
    double d_m = (r_m + l_m) * 0.5 - last_avg_;
    last_avg_ = (r_m + l_m) * 0.5;
    pose_.yaw_rad = yaw_deg * M_PI / 180.0;
    pose_.x_m += d_m * std::cos(pose_.yaw_rad);
    pose_.y_m += d_m * std::sin(pose_.yaw_rad);
    return pose_;
  }

  Pose2d pose() const { return pose_; }

 private:
  double wc_;
  double tw_;
  double last_avg_{0.0};
  Pose2d pose_;
};

}  // namespace vexlib::localization
