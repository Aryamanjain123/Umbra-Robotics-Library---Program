#include <gtest/gtest.h>
#include "vexlib/control/trajectory_follower.hpp"
#include "vexlib/motion/profile.hpp"

using namespace vexlib::control;
using namespace vexlib::motion;

// Simple 1D plant: y' = (-y + K*u)/tau  (u interpreted as "voltage")
struct FirstOrder {
  double y{0.0};
  double K{1.0};
  double tau{0.2};
  double step(double u, double dt) {
    double dydt = (-y + K*u) / tau;
    y += dydt * dt;
    return y;
  }
};

TEST(TrajectoryFollower, TrapezoidConvergesAndSettles) {
  // Constraints + profile
  ProfileConstraints pc{.max_vel=10.0, .max_acc=6.0};
  auto prof = std::make_shared<TrapezoidProfile>(0.0, 50.0, pc);

  // PID+FF config
  FollowerConfig fc;
  fc.pid.gains = {0.8, 1.8, 0.02, 0.3, 0.02, 0.004};
  fc.pid.limits = {-12.0, 12.0, -3.0, 3.0, 500.0};
  fc.pid.settle = {0.05, 0.10, 250, 5000};
  fc.pid.ff_enabled = true;
  fc.lookahead_end_s = 0.08;
  fc.max_run_ms = 8000;

  TrajectoryFollower follower(fc);
  follower.setProfile(prof);

  FirstOrder plant;
  plant.K = 1.0; plant.tau = 0.15;

  follower.start(plant.y);

  const double dt = 0.01;
  for (int i = 0; i < 1200; ++i) {
    double u = follower.update(plant.y, dt);
    plant.step(u, dt);
    if (follower.isDone()) break;
  }

  EXPECT_TRUE(follower.exitState().settled);
  EXPECT_FALSE(follower.exitState().timed_out);
  EXPECT_NEAR(plant.y, 50.0, 0.10);
}

TEST(TrajectoryFollower, CancelStopsEarly) {
  ProfileConstraints pc{.max_vel=6.0, .max_acc=3.0};
  auto prof = std::make_shared<TrapezoidProfile>(0.0, 30.0, pc);

  FollowerConfig fc;
  fc.pid.gains = {0.9, 1.5, 0.03, 0.2, 0.015, 0.003};
  fc.pid.limits = {-12, 12, -2, 2, 300};
  fc.pid.settle = {0.05, 0.10, 200, 4000};
  fc.pid.ff_enabled = true;

  TrajectoryFollower follower(fc);
  follower.setProfile(prof);

  FirstOrder plant;
  follower.start(plant.y);

  const double dt = 0.01;
  for (int i = 0; i < 150; ++i) {
    double u = follower.update(plant.y, dt);
    plant.step(u, dt);
    if (i == 100) follower.cancel();
  }

  EXPECT_TRUE(follower.exitState().canceled);
}
