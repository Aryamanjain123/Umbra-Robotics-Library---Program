#include <gtest/gtest.h>
#include "vexlib/control/pid.hpp"
#include <cmath>

// First-order plant: y' = (-y + K*u) / tau
struct FirstOrderPlant {
  double y{0.0};
  double K{1.0};
  double tau{0.2}; // seconds

  double step(double u, double dt) {
    const double dydt = (-y + K * u) / tau;
    y += dydt * dt;
    return y;
  }
};

TEST(PID_Settling, ReachesBandBeforeTimeout) {
  using namespace vexlib::control;

  PIDConfig cfg;
  cfg.gains = {0.9, 2.0, 0.02, 0,0,0};
  cfg.limits = {-12.0, 12.0, -2.0, 2.0, 200.0}; // 200 units/s slew
  cfg.settle = {0.01, 0.05, 250, 3000};         // 10ms loop → dwell 250ms, timeout 3s
  cfg.anti_windup = AntiWindupMode::BackCalculation;
  cfg.d_alpha = 0.3;
  cfg.i_leak_per_s = 0.0;

  PID pid(cfg);
  pid.setTarget(1.0);
  pid.reset(0.0);

  FirstOrderPlant plant;
  plant.K = 1.0;
  plant.tau = 0.15;

  const double dt = 0.01;
  for (int i = 0; i < 600; ++i) {
    const double u = pid.update(plant.y, dt);
    plant.step(u, dt);
    if (pid.isDone()) break;
  }

  EXPECT_FALSE(pid.isTimedOut());
  EXPECT_TRUE(pid.isSettled());
  EXPECT_NEAR(plant.y, 1.0, 0.02);
}

TEST(PID_Settling, TimesOutIfUnreachable) {
  using namespace vexlib::control;

  PIDConfig cfg;
  cfg.gains = {1.2, 0.6, 0.01, 0,0,0};
  cfg.limits = {0.0, 0.1, -0.5, 0.5, 50.0}; // very small output cap
  cfg.settle = {0.005, 0.05, 200, 1500};   // tight timeout
  cfg.anti_windup = AntiWindupMode::Clamp;

  PID pid(cfg);
  pid.setTarget(2.0);
  pid.reset(0.0);

  FirstOrderPlant plant;
  plant.K = 1.0;
  plant.tau = 0.25;

  const double dt = 0.01;
  for (int i = 0; i < 500; ++i) {
    const double u = pid.update(plant.y, dt);
    plant.step(u, dt);
    if (pid.isDone()) break;
  }

  EXPECT_TRUE(pid.isTimedOut());
  EXPECT_FALSE(pid.isSettled());
}
