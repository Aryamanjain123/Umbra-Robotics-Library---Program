#include "vexlib/drive/chassis.hpp"
#include "vexlib/localization/odometry3w.hpp"

namespace vexlib {
namespace drive {

localization::Pose2D Chassis::getPose() const {
  if (odom3w_) return odom3w_->pose();
  return {};
}

void Chassis::resetPose(const localization::Pose2D& p) {
  if (odom3w_) odom3w_->resetPose(p);
}

void Chassis::fuseAbsolutePose(const localization::Pose2D& p, double pos_std_m, double yaw_std_rad) {
  if (odom3w_) odom3w_->fuseAbsolutePose(p, pos_std_m, yaw_std_rad);
}

} // namespace drive
} // namespace vexlib
