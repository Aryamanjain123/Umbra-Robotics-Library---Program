#pragma once
#include <cmath>
#include <cstdint>

namespace vexlib {
namespace units {

struct Radians {
  double value{0.0};
  explicit constexpr Radians(double v = 0.0) : value(v) {}
};
struct Degrees {
  double value{0.0};
  explicit constexpr Degrees(double v = 0.0) : value(v) {}
  constexpr operator Radians() const { return Radians{value * M_PI / 180.0}; }
};

struct Meters {
  double value{0.0};
  explicit constexpr Meters(double v = 0.0) : value(v) {}
};
struct Millimeters {
  double value{0.0};
  explicit constexpr Millimeters(double v = 0.0) : value(v) {}
  constexpr operator Meters() const { return Meters{value / 1000.0}; }
};

struct Volts {
  double value{0.0};
  explicit constexpr Volts(double v = 0.0) : value(v) {}
};

struct Seconds {
  double value{0.0};
  explicit constexpr Seconds(double v = 0.0) : value(v) {}
};

}  // namespace units
}  // namespace vexlib
