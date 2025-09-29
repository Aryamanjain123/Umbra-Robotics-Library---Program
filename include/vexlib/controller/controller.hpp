#pragma once
#if __has_include("pros/controller.hpp")
  #include "pros/controller.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::io {

class Controller {
 public:
  Controller()
#if VEXLIB_HAS_PROS
    : ctrl_(pros::E_CONTROLLER_MASTER)
#endif
  {}

  // Joystick helpers: return -1..1
  double left_y() const {
#if VEXLIB_HAS_PROS
    return ctrl_.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
#else
    return 0.0;
#endif
  }
  double right_x() const {
#if VEXLIB_HAS_PROS
    return ctrl_.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
#else
    return 0.0;
#endif
  }
  bool button_a() const {
#if VEXLIB_HAS_PROS
    return ctrl_.get_digital(pros::E_CONTROLLER_DIGITAL_A);
#else
    return false;
#endif
  }
  void rumble(const char* pattern) {
#if VEXLIB_HAS_PROS
    ctrl_.rumble(pattern);
#else
    (void)pattern;
#endif
  }

 private:
#if VEXLIB_HAS_PROS
  pros::Controller ctrl_;
#endif
};

} // namespace vexlib::io
