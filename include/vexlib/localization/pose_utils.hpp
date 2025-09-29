#pragma once
#include <cmath>
#include "vexlib/localization/pose2d.hpp"

namespace vexlib {
namespace localization {

// SE(2) utility functions for Pose2D

// Relative transform: pose_B in pose_A's frame
inline Pose2D relativeTo(const Pose2D& A, const Pose2D& B) {
  // A^-1 ∘ B
  Pose2D inv = A.inverse();
  return inv.transformBy(B);
}

// Mirror across the Y axis (useful for symmetric autons)
inline Pose2D mirrorY(const Pose2D& p) {
  return Pose2D{-p.x_m, p.y_m, Pose2D::wrapRad(M_PI - p.theta_rad)};
}

// Mirror across the X axis
inline Pose2D mirrorX(const Pose2D& p) {
  return Pose2D{p.x_m, -p.y_m, Pose2D::wrapRad(-p.theta_rad)};
}

// Apply a twist (Δx, Δy, Δθ) in robot frame
inline Pose2D applyTwist(const Pose2D& base, double dx, double dy, double dtheta) {
  Pose2D delta{dx, dy, dtheta};
  return base.transformBy(delta);
}

// Offset pose by another pose (common for auton resets)
inline Pose2D offsetPose(const Pose2D& base, const Pose2D& offset) {
  return base.transformBy(offset);
}

}  // namespace localization
}  // namespace vexlib
