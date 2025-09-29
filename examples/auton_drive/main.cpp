
#include "vex.h"
#include "vexlib/drive/chassis.hpp"
#include "vexlib/localization/pose2d.hpp"

using namespace vex;
using namespace vexlib;

int main() {
  vexcodeInit();

  hal::Motor left(PORT1, false), right(PORT10, true);
  hal::Rotation lRot(PORT2), rRot(PORT3), sRot(PORT4);
  hal::IMU imu(PORT5);

  drive::Chassis chassis(left, right, lRot, rRot, sRot, imu);
  chassis.resetPose(localization::Pose2D{0,0,0});

  // Drive forward 24"
  chassis.driveTo(24_in);

  // Turn 90 deg
  chassis.turnTo(90_deg);

  // Drive to a pose on the field
  chassis.driveToPose(36_in, 24_in, 0_deg);
}
