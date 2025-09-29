#include <gtest/gtest.h>
#include "vexlib/motion/profile.hpp"
using namespace vexlib::motion;

TEST(Profile, TrapezoidEndsAtGoal) {
  ProfileConstraints c{.max_vel=10.0, .max_acc=5.0};
  TrapezoidProfile prof(0.0, 100.0, c);

  auto final = prof.sample(prof.duration());
  EXPECT_NEAR(final.pos, 100.0, 1e-2);
  EXPECT_NEAR(final.vel, 0.0, 1e-2);
}

TEST(Profile, SCurveEndsAtGoal) {
  ProfileConstraints c{.max_vel=8.0, .max_acc=4.0, .max_jerk=10.0};
  SCurveProfile prof(0.0, 50.0, c);

  auto final = prof.sample(prof.duration());
  EXPECT_NEAR(final.pos, 50.0, 1.0);  // approx model
}

TEST(Profile, TrapezoidMidpointVelocity) {
  ProfileConstraints c{.max_vel=6.0, .max_acc=3.0};
  TrapezoidProfile prof(0.0, 36.0, c);

  auto mid = prof.sample(prof.duration()/2);
  EXPECT_LE(std::fabs(mid.vel), c.max_vel + 1e-6);
}
