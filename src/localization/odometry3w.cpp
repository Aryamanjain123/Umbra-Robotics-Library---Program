#include "vexlib/localization/odometry3w.hpp"
#include <cmath>

namespace vexlib {
namespace localization {

Odometry3W::Odometry3W(const Geometry& geom,
                       hal::Rotation& left,
                       hal::Rotation& right,
                       hal::Rotation& lateral,
                       hal::IMU& imu,
                       const Config& cfg)
  : geom_(geom),
    cfg_(cfg),
    left_(left), right_(right), lateral_(lateral), imu_(imu),
    yaw_filter_(YawComplementary::Config{cfg.imu_alpha, M_PI/180.0}) {}

void Odometry3W::initialize(const Pose2D& initial_pose) {
  // Zero encoders so the first update reads pure deltas
  left_.resetPosition();
  right_.resetPosition();
  lateral_.resetPosition();

  // IMU might still be calibrating outside; we align our yaw estimate to IMU reading here.
  const double imu_deg = imu_.yaw_deg();
  yaw_filter_.reset(initial_pose.theta_rad, imu_deg);

  pose_ = initial_pose;
  initialized_ = true;
}

double Odometry3W::ticksToMeters(int32_t ticks) const {
  const double revs = static_cast<double>(ticks) / static_cast<double>(geom_.ticks_per_rev);
  return revs * (2.0 * M_PI * geom_.wheel_radius_m);
}

int32_t Odometry3W::readAndZero(hal::Rotation& enc) {
  const int32_t t = enc.position_ticks();
  enc.resetPosition();
  return t;
}

void Odometry3W::update(double /*dt_sec*/) {
  if (!initialized_) {
    initialize(Pose2D{0,0,0});
  }

  // Read and zero tick deltas each cycle
  int32_t dL_ticks = readAndZero(left_);
  int32_t dR_ticks = readAndZero(right_);
  int32_t dS_ticks = readAndZero(lateral_);

  if (geom_.invert_left)   dL_ticks = -dL_ticks;
  if (geom_.invert_right)  dR_ticks = -dR_ticks;
  if (geom_.invert_lateral)dS_ticks = -dS_ticks;

  const double dL = ticksToMeters(dL_ticks);
  const double dR = ticksToMeters(dR_ticks);
  const double dS = ticksToMeters(dS_ticks);

  // Core 3-wheel kinematics
  const double dtheta = (dR - dL) / geom_.track_width_m;        // wheel-derived heading delta
  const double s      = (dR + dL) * 0.5;                        // forward arc length
  const double yloc   = dS - geom_.lateral_offset_m * dtheta;   // lateral with offset correction

  // Midpoint heading for better integration
  const double theta_mid = Pose2D::wrapRad(pose_.theta_rad + 0.5 * dtheta);
  const double c = std::cos(theta_mid);
  const double sgn = std::sin(theta_mid);

  const double dx = c * s - sgn * yloc;
  const double dy = sgn * s + c  * yloc;

  // Update heading with IMU fusion
  const double imu_deg = imu_.yaw_deg();
  const double fused_heading = yaw_filter_.update(dtheta, imu_deg);

  pose_.x_m       += dx;
  pose_.y_m       += dy;
  pose_.theta_rad  = fused_heading;

  // Telemetry
  telem_.dL_m = dL;
  telem_.dR_m = dR;
  telem_.dS_m = dS;
  telem_.dtheta_rad = dtheta;
  telem_.dx_m = dx;
  telem_.dy_m = dy;
  telem_.heading_rad = pose_.theta_rad;
}

void Odometry3W::resetPose(const Pose2D& p) {
  pose_ = p;
  // Re-align IMU absolute to this heading so the complementary filter doesn't "snap" later.
  yaw_filter_.reset(p.theta_rad, imu_.yaw_deg());
  // Also zero encoders to remove residual deltas.
  left_.resetPosition();
  right_.resetPosition();
  lateral_.resetPosition();
}

void Odometry3W::fuseAbsolutePose(const Pose2D& absolute_pose, double pos_std_m, double yaw_std_rad) {
  // EKF-lite: single-step covariance-weighted blend.
  // Treat current estimate as z0 ~ N(pose,  Σ0), measurement as z1 ~ N(abs, Σ1).
  // Here we approximate Σ0 with moderate uncertainty; user supplies Σ1 via std args.
  const double sigma0_pos = 0.05;  // 5cm nominal trust in dead-reckoned position
  const double sigma0_yaw = 2.0 * M_PI / 180.0; // ~2deg nominal trust in yaw after fusion

  const double w_pos = sigma0_pos * sigma0_pos / (sigma0_pos*sigma0_pos + pos_std_m*pos_std_m);
  const double w_yaw = sigma0_yaw * sigma0_yaw / (sigma0_yaw*sigma0_yaw + yaw_std_rad*yaw_std_rad);

  pose_.x_m = (1.0 - w_pos) * pose_.x_m + w_pos * absolute_pose.x_m;
  pose_.y_m = (1.0 - w_pos) * pose_.y_m + w_pos * absolute_pose.y_m;

  const double dth = Pose2D::wrapRad(absolute_pose.theta_rad - pose_.theta_rad);
  pose_.theta_rad = Pose2D::wrapRad(pose_.theta_rad + w_yaw * dth);

  // Keep complementary filter consistent with the corrected heading.
  yaw_filter_.reset(pose_.theta_rad, imu_.yaw_deg());
}

}  // namespace localization
}  // namespace vexlib
