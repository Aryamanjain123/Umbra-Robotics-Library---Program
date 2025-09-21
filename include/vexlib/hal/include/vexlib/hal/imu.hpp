#pragma once
#include <cstdint>
#if __has_include("pros/imu.hpp")
  #include "pros/imu.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::hal {

class IMU {
 public:
  explicit IMU(int port): port_(port)
#if VEXLIB_HAS_PROS
    , imu_(port)
#endif
  {}
  double yaw_deg() const {
#if VEXLIB_HAS_PROS
    return imu_.get_yaw();
#else
    return 0.0;
#endif
  }
  double pitch_deg() const {
#if VEXLIB_HAS_PROS
    return imu_.get_pitch();
#else
    return 0.0;
#endif
  }
  double roll_deg() const {
#if VEXLIB_HAS_PROS
    return imu_.get_roll();
#else
    return 0.0;
#endif
  }
  void reset() {
#if VEXLIB_HAS_PROS
    imu_.reset();
#endif
  }

 private:
  int port_;
#if VEXLIB_HAS_PROS
  pros::Imu imu_;
#endif
};

}  // namespace vexlib::hal
