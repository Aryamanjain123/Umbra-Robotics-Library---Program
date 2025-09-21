#pragma once
#if __has_include("pros/adi.hpp")
  #include "pros/adi.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::hal::adi {

class Bumper {
 public:
  explicit Bumper(char port)
#if VEXLIB_HAS_PROS
    : bumper_(port)
#endif
  {}
  bool pressed() const {
#if VEXLIB_HAS_PROS
    return bumper_.get_value() == 1;
#else
    return false;
#endif
  }
 private:
#if VEXLIB_HAS_PROS
  pros::ADIDigitalIn bumper_;
#endif
};

} // namespace vexlib::hal::adi
