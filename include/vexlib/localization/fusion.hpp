#pragma once
#include <cstdint>
#include "vexlib/localization/pose2d.hpp"

namespace vexlib {
namespace localization {

// Minimal complementary filter for yaw (wheel odom + IMU absolute yaw).
class YawComplementary {
public:
  struct Config {
    double alpha = 0.96;          // weight on integrated (wheel) yaw
    double imu_deg_to_rad = M_PI / 180.0;
  };

  explicit YawComplementary(const Config& cfg = Config()) : cfg_(cfg) {}

  void reset(double heading_rad, double imu_heading_deg) {
    imu_align_rad_ = heading_rad - (imu_heading_deg * cfg_.imu_deg_to_rad);
    est_heading_rad_ = Pose2D::wrapRad(heading_rad);
    initialized_ = true;
  }

  // wheel_delta_rad: change from wheel odometry this cycle.
  // imu_heading_deg: absolute IMU yaw (deg) this cycle.
  double update(double wheel_delta_rad, double imu_heading_deg) {
    if (!initialized_) {
      reset(0.0, imu_heading_deg);
    }

    const double imu_abs_rad = imu_heading_deg * cfg_.imu_deg_to_rad + imu_align_rad_;
    const double wheel_pred  = Pose2D::wrapRad(est_heading_rad_ + wheel_delta_rad);

    // Complementary blend: keep short-term wheel detail, long-term IMU absolute.
    const double fused = Pose2D::wrapRad(cfg_.alpha * wheel_pred + (1.0 - cfg_.alpha) * imu_abs_rad);
    est_heading_rad_ = fused;
    return est_heading_rad_;
  }

  double heading() const { return est_heading_rad_; }

  void setAlpha(double a) { cfg_.alpha = a; }
  double alpha() const { return cfg_.alpha; }

private:
  Config cfg_;
  bool initialized_{false};
  double imu_align_rad_{0.0};
  double est_heading_rad_{0.0};
};

}  // namespace localization
}  // namespace vexlib
