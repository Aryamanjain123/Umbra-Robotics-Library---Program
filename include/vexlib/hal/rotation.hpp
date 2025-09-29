#pragma once
#if __has_include("pros/rotation.hpp")
  #include "pros/rotation.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::hal {
class Rotation {
 public:
  explicit Rotation(int port): port_(port)
#if VEXLIB_HAS_PROS
    , rot_(port)
#endif
  {}
  double position_deg() const {
#if VEXLIB_HAS_PROS
    return rot_.get_position() / 100.0;
#else
    return 0.0;
#endif
  }
  double velocity_dps() const {
#if VEXLIB_HAS_PROS
    return rot_.get_velocity();
#else
    return 0.0;
#endif
  }
  void reset() {
#if VEXLIB_HAS_PROS
    rot_.reset_position();
#endif
  }
 private:
  int port_;
#if VEXLIB_HAS_PROS
  pros::Rotation rot_;
#endif
};
} // namespace vexlib::hal
