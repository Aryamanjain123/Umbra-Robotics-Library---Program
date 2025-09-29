#include "vexlib/control/trajectory_follower.hpp"
#include <cmath>

namespace vexlib {
namespace control {

TrajectoryFollower::TrajectoryFollower(const FollowerConfig& cfg)
  : cfg_(cfg), pid_(cfg.pid) {}

void TrajectoryFollower::setProfile(std::shared_ptr<motion::MotionProfile> profile) {
  profile_ = std::move(profile);
  if (profile_) {
    duration_ = profile_->duration();
    // best-effort guess at final target for hold phase
    auto final = profile_->sample(duration_);
    target_end_ = final.pos;
  } else {
    duration_ = 0.0;
    target_end_ = 0.0;
  }
}

void TrajectoryFollower::start(double current_measured) {
  exit_ = {};
  t_ = 0.0;
  run_ms_ = 0;
  paused_ = false;
  active_ = (profile_ != nullptr);

  if (!active_) return;
  pid_.reset(current_measured);
  if (onStart) onStart();
}

void TrajectoryFollower::pause() { paused_ = true; }
void TrajectoryFollower::resume() { paused_ = false; }
void TrajectoryFollower::cancel() {
  exit_.canceled = true;
  active_ = false;
  if (onCancel) onCancel();
}

void TrajectoryFollower::evaluateExit(double dt_s) {
  // Update hard run cap
  if (dt_s > 0.0) run_ms_ += static_cast<int64_t>(dt_s * 1000.0);

  // Profile finished?
  if (profile_ && t_ >= duration_ && !exit_.profile_done) {
    exit_.profile_done = true;
    if (onFinish) onFinish();
  }

  // PID settle/timeout propagate to follower state
  if (pid_.isSettled() && !exit_.settled) {
    exit_.settled = true;
    active_ = false;
    if (onSettle) onSettle();
  }
  if (pid_.isTimedOut() && !exit_.timed_out) {
    exit_.timed_out = true;
    active_ = false;
    if (onTimeout) onTimeout();
  }

  // Hard cap timeout
  if (cfg_.max_run_ms > 0 && run_ms_ >= cfg_.max_run_ms) {
    exit_.timed_out = true;
    active_ = false;
    if (onTimeout) onTimeout();
  }
}

double TrajectoryFollower::update(double measured, double dt_s) {
  if (!active_ || !profile_) return 0.0;
  if (paused_) {
    // While paused, keep PID target at current measurement to avoid windup
    pid_.setTarget(measured);
    return pid_.update(measured, dt_s);
  }

  // Advance local time
  if (dt_s > 0.0) t_ += dt_s;

  // Sample profile or hold final
  motion::ProfileState ref{};
  if (t_ <= duration_) {
    ref = profile_->sample(t_);
  } else {
    ref = profile_->sample(duration_);
    // Optionally extend a little to hold target for dwell time
    if (cfg_.hold_final_target && t_ <= duration_ + cfg_.lookahead_end_s) {
      ref.pos = target_end_;
      ref.vel = 0.0;
      ref.acc = 0.0;
    }
  }

  // Feed desired state into PID+FF
  pid_.setTarget(ref.pos);
  pid_.setFFState(ref.vel, ref.acc);
  double u = pid_.update(measured, dt_s);

  // Evaluate exit logic
  evaluateExit(dt_s);
  return u;
}

} // namespace control
} // namespace vexlib
