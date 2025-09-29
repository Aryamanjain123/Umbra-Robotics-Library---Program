#pragma once
#include <cstdint>
#include "vexlib/localization/pose2d.hpp"
#include "vexlib/hal/rotation.hpp"
#include "vexlib/hal/imu.hpp"
#include "vexlib/localization/fusion.hpp"

namespace vexlib {
namespace localization {

// 3-wheel odometry with IMU yaw fusion (complementary filter).
class Odometry3W {
public:
  struct Geometry {
    // Track and wheel geometry in meters.
    double track_width_m;      // distance between left & right tracking wheels
    double lateral_offset_m;   // signed y-offset of lateral wheel from robot center (+left)
    double wheel_radius_m;     // radius of tracking wheels
    int    ticks_per_rev;      // encoder counts per revolution (Rotation sensor -> set accordingly)
    bool   invert_left{false};
    bool   invert_right{false};
    bool   invert_lateral{false};
  };

  struct Config {
    double imu_alpha = 0.96;     // yaw complementary alpha
    double s_eps_m = 1e-12;      // numerical stability on small arcs
  };

  struct Telemetry {
    double dL_m{0}, dR_m{0}, dS_m{0};
    double dtheta_rad{0};
    double dx_m{0}, dy_m{0};
    double heading_rad{0};
  };

  Odometry3W(const Geometry& geom,
             hal::Rotation& left,
             hal::Rotation& right,
             hal::Rotation& lateral,
             hal::IMU& imu,
             const Config& cfg = Config());

  // Call once before use. Zeroes deltas and aligns IMU to current heading.
  void initialize(const Pose2D& initial_pose = Pose2D{0,0,0});

  // Call every loop with dt (seconds) OR call frequently without dt (dt not used by algo).
  void update(double /*dt_sec*/);

  Pose2D pose() const { return pose_; }
  void   resetPose(const Pose2D& p);

  // Optionally nudge pose with an absolute fix (e.g., VEX GPS) with Kalman-lite weighting.
  void fuseAbsolutePose(const Pose2D& absolute_pose, double pos_std_m, double yaw_std_rad);

  // Expose telemetry for debugging.
  Telemetry lastTelemetry() const { return telem_; }

private:
  // Helpers
  double ticksToMeters(int32_t ticks) const;
  int32_t readAndZero(hal::Rotation& enc);

  Geometry geom_;
  Config   cfg_;

  hal::Rotation& left_;
  hal::Rotation& right_;
  hal::Rotation& lateral_;
  hal::IMU&      imu_;

  YawComplementary yaw_filter_;

  Pose2D  pose_;
  Telemetry telem_;

  // Encoder residuals (we zero encoders each cycle for clean deltas).
  bool initialized_{false};
};

}  // namespace localization
}  // namespace vexlib
