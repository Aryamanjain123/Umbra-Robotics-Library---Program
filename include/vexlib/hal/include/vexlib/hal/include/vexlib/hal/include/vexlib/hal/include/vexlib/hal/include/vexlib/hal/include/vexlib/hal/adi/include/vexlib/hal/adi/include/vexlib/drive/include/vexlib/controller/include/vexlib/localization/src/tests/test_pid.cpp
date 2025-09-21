
#include "vexlib/control/pid.hpp"
#include <gtest/gtest.h>

TEST(PID, StepResponse) {
  vexlib::control::PID pid(1.0, 0.0, 0.0, 12.0);
  double out = 0;
  double err = 5.0;
  out = pid.update(err, 0.02);
  EXPECT_GT(out, 0.0);
  EXPECT_LE(out, 12.0);
}
