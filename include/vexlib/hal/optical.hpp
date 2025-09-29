#pragma once
#if __has_include("pros/optical.hpp")
  #include "pros/optical.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::hal {

class Optical {
 public:
  explicit Optical(int port): port_(port)
#if VEXLIB_HAS_PROS
    , opt_(port)
#endif
  {}
  double hue() const {
#if VEXLIB_HAS_PROS
    return opt_.get_hue();
#else
    return 0.0;
#endif
  }
  double proximity() const {
#if VEXLIB_HAS_PROS
    return opt_.get_proximity();
#else
    return 0.0;
#endif
  }
 private:
  int port_;
#if VEXLIB_HAS_PROS
  pros::Optical opt_;
#endif
};

} // namespace vexlib::hal
