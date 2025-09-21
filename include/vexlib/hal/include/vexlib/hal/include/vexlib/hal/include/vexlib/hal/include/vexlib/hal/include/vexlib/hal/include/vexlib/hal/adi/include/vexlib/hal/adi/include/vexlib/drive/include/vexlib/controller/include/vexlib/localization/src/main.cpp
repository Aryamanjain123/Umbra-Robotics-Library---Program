// If you're compiling on the robot with PROS, this file becomes your entry point.
// You can merge parts of this into an existing project as needed.

#if __has_include("pros/apix.h")
#include "pros/apix.h"
#include "pros/rtos.hpp"
#include "vexlib/drive/chassis.hpp"
#include "vexlib/hal/imu.hpp"
#include "vexlib/hal/motor.hpp"
#include "vexlib/logger.hpp"
#include "vexlib/controller/controller.hpp"

vexlib::hal::Motor leftMotor(1, false, 200.0);
vexlib::hal::Motor rightMotor(2, true, 200.0);
vexlib::drive::TankChassis chassis(leftMotor, rightMotor);
vexlib::hal::IMU imu(3);
vexlib::io::Controller master;

void initialize() {
  vexlib::Logger::set_level(vexlib::LogLevel::kInfo);
  vexlib::Logger::log(vexlib::LogLevel::kInfo, "init", "vexlib %s initializing...", vexlib::VERSION);
  imu.reset();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // Simple demo: drive forward for 1s
  auto t0 = pros::millis();
  while (pros::millis() - t0 < 1000) {
    chassis.arcade(0.5, 0.0);
    pros::delay(10);
  }
  chassis.arcade(0.0, 0.0);
}

void opcontrol() {
  while (true) {
    double fwd = -master.left_y();     // invert so up is positive
    double turn = master.right_x();
    bool quick = master.button_a();
    chassis.curvature(fwd, turn, quick);
    pros::delay(10);
  }
}

#else
int main() {
  // Host build (no PROS). Nothing to do here; run unit tests instead.
  return 0;
}
#endif
