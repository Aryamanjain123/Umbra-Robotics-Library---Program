#pragma once
#include <memory>
#include <functional>
#include <cstdint>
#include <algorithm>
#include "vexlib/control/pid.hpp"
#include "vexlib/motion/profile.hpp"

namespace vexlib {
namespace control {

struct FollowerExit {
  bool settled{false};
  bool timed_out{false};
  bool canceled{false};
  bool profile_done{false}; // profile finished sampling window
  bool done() const { return settled || timed_out || canceled; }
};

struct FollowerConfig {
  PIDConfig pid;                 // PID+FF configuration (limits, settle, etc.)
  double lookahead_end_s{0.050}; // extra time to hold final target after profile ends
  bool hold_final_target{true};  // hold final pose during dwell
  int max_run_ms{0};             // hard cap; 0 = disabled
};

class TrajectoryFollower {
public:
  explicit TrajectoryFollower(const FollowerConfig& cfg);

  // Provide a profile to follow (ownership shared so you can keep references if needed)
  void setProfile(std::shared_ptr<motion::MotionProfile> profile);

  // Start following from current measured position; resets internal PID and timers.
  void start(double current_measured);

  // Update once per loop with measured position and dt (seconds).
  // Returns actuator command (e.g., volts) AFTER limits/slew inside PID.
  double update(double measured, double dt_s);

  // Control flow
  void pause();
  void resume();
  void cancel();

  // Status
  bool isActive() const { return active_ && !exit_.done(); }
  bool isPaused() const { return paused_; }
  const FollowerExit& exitState() const { return exit_; }
  bool isDone() const { return exit_.done(); }
  bool isSettled() const { return exit_.settled; }

  // Telemetry
  double elapsed() const { return t_; }
  double duration() const { return duration_; }
  double lastOutput() const { return pid_.lastOutput(); }
  double lastError()  const { return pid_.lastError(); }
  double lastFF()     const { return pid_.lastFF(); }
  double targetAtEnd() const { return target_end_; }

  // Optional event callbacks (set these if you want notifications)
  std::function<void()> onStart;
  std::function<void()> onSettle;
  std::function<void()> onTimeout;
  std::function<void()> onCancel;
  std::function<void()> onFinish; // profile finished (before settle)

  // Access to the internal PID for live tuning (safe to call between updates)
  PID& pid() { return pid_; }

private:
  FollowerConfig cfg_;
  PID pid_;
  std::shared_ptr<motion::MotionProfile> profile_;
  double duration_{0.0};
  double target_end_{0.0};

  // State
  bool active_{false};
  bool paused_{false};
  FollowerExit exit_{};
  double t_{0.0};           // seconds since start
  int64_t run_ms_{0};       // hard cap stopwatch

  void evaluateExit(double dt_s);
};

} // namespace control
} // namespace vexlib
