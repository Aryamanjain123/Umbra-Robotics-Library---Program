#include "vex.h"

// Core drive & odometry
#include "vexlib/drive/chassis.hpp"
#include "vexlib/localization/pose2d.hpp"

// Telemetry (optional)
#include "vexlib/telemetry/telemetry.hpp"

using namespace vex;
using namespace vexlib;

int main() {
  vexcodeInit();

  // -----------------------------
  // 1. Hardware setup (user wires their ports)
  // -----------------------------
  hal::Motor leftMotor(PORT1, false);     // left drive motor
  hal::Motor rightMotor(PORT10, true);    // right drive motor
  hal::Rotation leftOdom(PORT2);          // left tracking wheel
  hal::Rotation rightOdom(PORT3);         // right tracking wheel
  hal::Rotation lateralOdom(PORT4);       // lateral tracking wheel
  hal::IMU imu(PORT5);                    // IMU

  // -----------------------------
  // 2. High-level chassis
  // -----------------------------
  drive::Chassis chassis(leftMotor, rightMotor,
                         leftOdom, rightOdom, lateralOdom, imu);

  // Reset pose to field origin
  chassis.resetPose(localization::Pose2D{0.0, 0.0, 0.0});

  // -----------------------------
  // 3. Telemetry setup (optional)
  // -----------------------------
  telemetry::TelemetryBuffer buf;
  telemetry::CSVLogger logger;
  logger.open("auton.csv");                // will save to SD card if present
  chassis.setTelemetryBuffer(&buf);
  chassis.setCSVLogger(&logger);

  // -----------------------------
  // 4. Autonomous routine
  // -----------------------------
  chassis.driveTo(24_in);                  // drive forward 24"
  chassis.turnTo(90_deg);                  // turn 90°
  chassis.driveToPose(36_in, 24_in, 0_deg);// drive to a field pose

  // Close log file (important!)
  logger.close();

  // -----------------------------
  // 5. Background loop
  // -----------------------------
  while (true) {
    chassis.update(0.02);  // update odometry & telemetry every 20ms

    // Print live pose to screen
    auto p = chassis.getPose();
    Brain.Screen.printAt(10, 40, "x=%.2f y=%.2f th=%.1f",
                         p.x_m, p.y_m, localization::Pose2D::rad2deg(p.theta_rad));

    this_thread::sleep_for(20);
  }
}
