#pragma once
#include <cmath>
#include <cstdint>
#include "vexlib/units.hpp"

namespace vexlib {
namespace localization {

// Right-handed field: +x forward, +y left, heading (rad) CCW, 0 rad facing +x.
struct Pose2D {
  double x_m{0.0};
  double y_m{0.0};
  double theta_rad{0.0};

  Pose2D() = default;
  Pose2D(double x, double y, double th) : x_m(x), y_m(y), theta_rad(th) {}

  static double wrapRad(double r) {
    // Wrap to (-pi, pi]
    while (r <= -M_PI) r += 2.0 * M_PI;
    while (r >   M_PI) r -= 2.0 * M_PI;
    return r;
  }

  Pose2D transformBy(const Pose2D& delta) const {
    const double c = std::cos(theta_rad);
    const double s = std::sin(theta_rad);
    const double gx = x_m + c * delta.x_m - s * delta.y_m;
    const double gy = y_m + s * delta.x_m + c * delta.y_m;
    const double gth = wrapRad(theta_rad + delta.theta_rad);
    return Pose2D{gx, gy, gth};
  }

  Pose2D inverse() const {
    const double c = std::cos(theta_rad);
    const double s = std::sin(theta_rad);
    // Inverse SE(2): R^T and -R^T t
    const double ix = -( c * x_m + s * y_m);
    const double iy = -(-s * x_m + c * y_m);
    return Pose2D{ix, iy, wrapRad(-theta_rad)};
  }

  // Convenience (degrees <-> radians) using your units.hpp literals if desired.
  static double deg2rad(double d) { return d * M_PI / 180.0; }
  static double rad2deg(double r) { return r * 180.0 / M_PI; }
};

}  // namespace localization
}  // namespace vexlib
