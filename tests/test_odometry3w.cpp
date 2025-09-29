#include <gtest/gtest.h>
#include "vexlib/localization/odometry3w.hpp"

// Minimal fakes for HAL (replace with your existing test doubles if you have them)
namespace vexlib { namespace hal {
struct FakeRotation : public Rotation {
  int32_t ticks{0};
  void resetPosition() override { ticks = 0; }
  int32_t position_ticks() override { return ticks; }
};
struct FakeIMU : public IMU {
  double yaw{0.0};
  double yaw_deg() override { return yaw; }
};
}} // ns

using namespace vexlib::localization;
using vexlib::hal::FakeRotation;
using vexlib::hal::FakeIMU;

TEST(Odometry3W, StraightLine) {
  FakeRotation L, R, S;
  FakeIMU imu;

  Odometry3W::Geometry geom{0.300, 0.100, 0.028, 360, false, false, false}; // 28mm tracking wheels, 360 tpr
  Odometry3W odo(geom, L, R, S, imu);
  odo.initialize(Pose2D{0,0,0});

  // Simulate 1 full rev on both L/R (approx 2*pi*r = 0.176 m)
  L.ticks = 360; R.ticks = 360; S.ticks = 0; imu.yaw = 0.0;
  odo.update(0.01);

  auto p = odo.pose();
  EXPECT_NEAR(p.theta_rad, 0.0, 1e-6);
  EXPECT_NEAR(p.x_m, 2.0 * M_PI * 0.028 * 0.5 * 2.0, 1e-3); // ~0.176 m
  EXPECT_NEAR(p.y_m, 0.0, 1e-6);
}

TEST(Odometry3W, PureRotation) {
  FakeRotation L, R, S;
  FakeIMU imu;

  Odometry3W::Geometry geom{0.300, 0.100, 0.028, 360, false, false, false};
  Odometry3W odo(geom, L, R, S, imu);
  odo.initialize(Pose2D{0,0,0});

  // Opposite ticks => rotate in place; IMU reports similar absolute yaw (~10 deg)
  L.ticks = -180; R.ticks = 180; S.ticks = 0; imu.yaw = 10.0;
  odo.update(0.01);

  auto p = odo.pose();
  // Wheel-only dtheta = (dR - dL)/track = ((+0.5rev) - (-0.5rev))*circum / track
  // Here we only assert it moved some heading and stayed near origin.
  EXPECT_NEAR(std::hypot(p.x_m, p.y_m), 0.0, 5e-3);
  EXPECT_NEAR(Pose2D::rad2deg(p.theta_rad), 10.0, 4.0); // fused toward IMU
}

TEST(Odometry3W, LateralOnly) {
  FakeRotation L, R, S;
  FakeIMU imu;

  Odometry3W::Geometry geom{0.300, 0.100, 0.028, 360, false, false, false};
  Odometry3W odo(geom, L, R, S, imu);
  odo.initialize(Pose2D{0,0,0});

  // Pure strafe: lateral wheel rolls, IMU unchanged
  L.ticks = 0; R.ticks = 0; S.ticks = 360; imu.yaw = 0.0;
  odo.update(0.01);

  auto p = odo.pose();
  EXPECT_NEAR(p.x_m, 0.0, 1e-6);
  EXPECT_NEAR(p.y_m, 2.0 * M_PI * 0.028, 1e-3);
}
