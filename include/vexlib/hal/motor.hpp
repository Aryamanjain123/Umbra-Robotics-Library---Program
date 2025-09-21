#include "vexlib/hal/motor.hpp"

namespace vexlib::hal {

Motor::Motor(int port, bool reversed, double gearset_rpm)
    : port_(port), reversed_(reversed), gear_rpm_(gearset_rpm)
#if VEXLIB_HAS_PROS
    , motor_(port_, reversed_)
#endif
{
#if VEXLIB_HAS_PROS
  // Gearset inference: 600 (blue), 200 (green), 100 (red)
  if (std::abs(gear_rpm_ - 600.0) < 1.0) motor_.set_gearing(pros::E_MOTOR_GEARSET_06);
  else if (std::abs(gear_rpm_ - 200.0) < 1.0) motor_.set_gearing(pros::E_MOTOR_GEARSET_18);
  else motor_.set_gearing(pros::E_MOTOR_GEARSET_36);
#endif
}

void Motor::set_voltage(double volts) {
#if VEXLIB_HAS_PROS
  motor_.move_voltage(static_cast<int16_t>(volts * 1000));
#else
  (void)volts;
#endif
}

void Motor::set_velocity_rpm(double rpm) {
#if VEXLIB_HAS_PROS
  motor_.move_velocity(static_cast<int16_t>(rpm));
#else
  (void)rpm;
#endif
}

double Motor::get_velocity_rpm() const {
#if VEXLIB_HAS_PROS
  return motor_.get_actual_velocity();
#else
  return 0.0;
#endif
}

double Motor::get_position_deg() const {
#if VEXLIB_HAS_PROS
  return motor_.get_position();
#else
  return 0.0;
#endif
}

void Motor::tare_position() {
#if VEXLIB_HAS_PROS
  motor_.tare_position();
#endif
}

void Motor::set_brake(BrakeMode mode) {
#if VEXLIB_HAS_PROS
  pros::motor_brake_mode_e_t m = pros::E_MOTOR_BRAKE_COAST;
  if (mode == BrakeMode::kBrake) m = pros::E_MOTOR_BRAKE_BRAKE;
  if (mode == BrakeMode::kHold) m = pros::E_MOTOR_BRAKE_HOLD;
  motor_.set_brake_mode(m);
#else
  (void)mode;
#endif
}

}  // namespace vexlib::hal
