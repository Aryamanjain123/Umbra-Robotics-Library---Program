#pragma once
#if __has_include("pros/adi.hpp")
  #include "pros/adi.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::hal::adi {

class LimitSwitch {
 public:
  explicit LimitSwitch(char port)
#if VEXLIB_HAS_PROS
    : limit_(port)
#endif
  {}
  bool pressed() const {
#if VEXLIB_HAS_PROS
    return limit_.get_value() == 1;
#else
    return false;
#endif
  }
 private:
#if VEXLIB_HAS_PROS
  pros::ADIDigitalIn limit_;
#endif
};

} // namespace vexlib::hal::adi
