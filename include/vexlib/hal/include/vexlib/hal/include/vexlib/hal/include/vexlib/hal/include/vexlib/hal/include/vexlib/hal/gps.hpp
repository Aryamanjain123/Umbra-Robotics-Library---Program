#pragma once
#if __has_include("pros/gps.hpp")
  #include "pros/gps.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::hal {

class GPS {
 public:
  explicit GPS(int port): port_(port)
#if VEXLIB_HAS_PROS
    , gps_(port)
#endif
  {}
  double x_mm() const {
#if VEXLIB_HAS_PROS
    return gps_.get_status().x;
#else
    return 0.0;
#endif
  }
  double y_mm() const {
#if VEXLIB_HAS_PROS
    return gps_.get_status().y;
#else
    return 0.0;
#endif
  }
  double yaw_deg() const {
#if VEXLIB_HAS_PROS
    return gps_.get_status().yaw;
#else
    return 0.0;
#endif
  }
 private:
  int port_;
#if VEXLIB_HAS_PROS
  pros::Gps gps_;
#endif
};

} // namespace vexlib::hal
