#pragma once
#include <algorithm>
#include <array>
#include <cmath>

#include "vexlib/hal/motor.hpp"
#include "vexlib/hal/rotation.hpp"
#include "vexlib/hal/imu.hpp"
#include "vexlib/localization/odometry3w.hpp"
#include "vexlib/localization/pose2d.hpp"

namespace vexlib::drive {

// ----------------------------------------------------------------------
// Low-level tank chassis (2-motor / 4-motor via motor groups)
// ----------------------------------------------------------------------
class TankChassis {
 public:
  // Provide left/right motors (they can be plugged into a motor group externally if desired)
  TankChassis(hal::Motor& left, hal::Motor& right, double max_volts = 12.0)
      : left_(left), right_(right), maxV_(max_volts) {}

  // Arcade drive (fwd [-1..1], turn [-1..1])
  void arcade(double fwd, double turn) {
    double l = std::clamp(fwd + turn, -1.0, 1.0);
    double r = std::clamp(fwd - turn, -1.0, 1.0);
    left_.set_voltage(l * maxV_);
    right_.set_voltage(r * maxV_);
  }

  // Curvature drive (CheesyDrive-ish)
  void curvature(double fwd, double curvature, bool allow_quickturn) {
    double angular = allow_quickturn ? curvature : std::abs(fwd) * curvature;
    arcade(fwd, angular);
  }

  void brake(hal::BrakeMode mode) {
    left_.set_brake(mode);
    right_.set_brake(mode);
  }

 private:
  hal::Motor& left_;
  hal::Motor& right_;
  double maxV_;
};

// ----------------------------------------------------------------------
// High-level competition chassis with Odometry++
// ----------------------------------------------------------------------
class Chassis {
 public:
  Chassis(hal::Motor& leftMotor,
          hal::Motor& rightMotor,
          hal::Rotation& leftOdom,
          hal::Rotation& rightOdom,
          hal::Rotation& lateralOdom,
          hal::IMU& imu)
      : tank_(leftMotor, rightMotor),
        odom_(geom_, leftOdom, rightOdom, lateralOdom, imu) {
    odom_.initialize();
  }

  // ---------------- Drive passthroughs ----------------
  void arcade(double fwd, double turn) { tank_.arcade(fwd, turn); }
  void curvature(double fwd, double curv, bool quick) { tank_.curvature(fwd, curv, quick); }
  void brake(hal::BrakeMode mode) { tank_.brake(mode); }

  // ---------------- Odometry passthroughs ----------------
  localization::Pose2D getPose() const { return odom_.pose(); }
  void resetPose(const localization::Pose2D& p) { odom_.resetPose(p); }
  void fuseAbsolutePose(const localization::Pose2D& p, double posStd, double yawStd) {
    odom_.fuseAbsolutePose(p, posStd, yawStd);
  }

  // Call this every 10–20 ms in your main loop
  void update(double dt_sec) { odom_.update(dt_sec); }

 private:
  TankChassis tank_;
  localization::Odometry3W odom_;

  // Geometry constants (tune these for your robot!)
  localization::Odometry3W::Geometry geom_{
    0.300,   // track width (m)
    0.100,   // lateral offset (m)
    0.028,   // tracking wheel radius (m)
    360,     // ticks per revolution
    false,   // invert left
    false,   // invert right
    false    // invert lateral
  }
  void setTelemetryBuffer(telemetry::TelemetryBuffer* buf) { telemBuf_ = buf; }
  void setCSVLogger(telemetry::CSVLogger* log) { logger_ = log; }

  void update(double dt_sec) {
    odom_.update(dt_sec);

    if (telemBuf_ || logger_) {
      telemetry::Sample s;
      s.t_sec = Brain.Timer.time(sec);
      auto p = getPose();
      s.x_m = p.x_m;
      s.y_m = p.y_m;
      s.theta_rad = p.theta_rad;
      if (telemBuf_) telemBuf_->push(s);
      if (logger_) logger_->log(s);
    }
  }

 private:
  TankChassis tank_;
  localization::Odometry3W odom_;
  localization::Odometry3W::Geometry geom_{
    0.300,0.100,0.028,360,false,false,false
  };

  telemetry::TelemetryBuffer* telemBuf_{nullptr};
  telemetry::CSVLogger* logger_{nullptr};

};
};

}  // namespace vexlib::drive
