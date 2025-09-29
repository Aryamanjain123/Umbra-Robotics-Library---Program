#include <gtest/gtest.h>
#include "vexlib/localization/pose_utils.hpp"

using namespace vexlib::localization;

TEST(PoseUtils, RelativeTo) {
  Pose2D A(1.0, 0.0, 0.0);
  Pose2D B(2.0, 0.0, 0.0);

  auto rel = relativeTo(A, B);
  EXPECT_NEAR(rel.x_m, 1.0, 1e-6);
  EXPECT_NEAR(rel.y_m, 0.0, 1e-6);
  EXPECT_NEAR(rel.theta_rad, 0.0, 1e-6);
}

TEST(PoseUtils, MirrorY) {
  Pose2D p(1.0, 2.0, 0.0);
  auto m = mirrorY(p);
  EXPECT_NEAR(m.x_m, -1.0, 1e-6);
  EXPECT_NEAR(m.y_m, 2.0, 1e-6);
}

TEST(PoseUtils, MirrorX) {
  Pose2D p(1.0, 2.0, M_PI/4);
  auto m = mirrorX(p);
  EXPECT_NEAR(m.x_m, 1.0, 1e-6);
  EXPECT_NEAR(m.y_m, -2.0, 1e-6);
}

TEST(PoseUtils, ApplyTwist) {
  Pose2D base(0.0, 0.0, 0.0);
  auto p = applyTwist(base, 1.0, 0.0, 0.0);
  EXPECT_NEAR(p.x_m, 1.0, 1e-6);
}

TEST(PoseUtils, OffsetPose) {
  Pose2D base(1.0, 1.0, 0.0);
  Pose2D offset(1.0, 0.0, M_PI/2);
  auto res = offsetPose(base, offset);
  EXPECT_NEAR(res.x_m, 2.0, 1e-6);
  EXPECT_NEAR(res.y_m, 1.0, 1e-6);
}
