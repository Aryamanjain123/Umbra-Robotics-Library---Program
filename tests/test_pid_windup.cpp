#include <gtest/gtest.h>
#include "vexlib/control/pid.hpp"

// Compare overshoot with Clamp vs BackCalculation

struct Plant {
  double y{0.0};
  double step(double u, double dt) {
    // simple critically-damped discrete-ish model
    const double a = 0.90;
    const double b = 0.10;
    y = a * y + b * u;
    return y;
  }
};

static double run_case(vexlib::control::PID& pid, double dt, int steps) {
  Plant p;
  p.y = 0.0;
  pid.reset(0.0);
  pid.setTarget(1.0);

  double max_overshoot = 0.0;
  for (int i = 0; i < steps; ++i) {
    double u = pid.update(p.y, dt);
    p.step(u, dt);
    max_overshoot = std::max(max_overshoot, p.y - 1.0);
  }
  return max_overshoot;
}

TEST(PID_Windup, BackCalculationReducesOvershoot) {
  using namespace vexlib::control;

  PIDConfig base;
  base.gains = {0.9, 2.5, 0.03, 0,0,0};
  base.limits = {-1.0, 1.0, -0.5, 0.5, 0.0};
  base.settle = {0.01, 0.05, 200, 0};

  PIDConfig clamp_cfg = base;
  clamp_cfg.anti_windup = AntiWindupMode::Clamp;

  PIDConfig back_cfg = base;
  back_cfg.anti_windup = AntiWindupMode::BackCalculation;

  PID pid_clamp(clamp_cfg);
  PID pid_back(back_cfg);

  const double dt = 0.01;
  const int steps = 400;

  double overshoot_clamp = run_case(pid_clamp, dt, steps);
  double overshoot_back  = run_case(pid_back , dt, steps);

  // Back-calculation should typically overshoot less or equal
  EXPECT_LE(overshoot_back, overshoot_clamp + 1e-6);
}
