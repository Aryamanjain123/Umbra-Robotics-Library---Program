#include <gtest/gtest.h>
#include "vexlib/localization/fusion.hpp"

using vexlib::localization::YawComplementary;

TEST(YawComplementary, ResetsAndFuses) {
  YawComplementary f({0.90, M_PI/180.0});
  f.reset(0.0, 0.0);
  // 20 deg IMU, wheel suggests 10 deg delta this cycle
  double h = f.update(YawComplementary::Config{}.imu_deg_to_rad * 10.0, 20.0);
  // Should be between 10 and 20 deg, closer to 10 due to alpha=0.9
  EXPECT_GT(h, 10.0 * M_PI/180.0);
  EXPECT_LT(h, 20.0 * M_PI/180.0);
}
