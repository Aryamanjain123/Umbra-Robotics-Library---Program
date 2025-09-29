#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>

namespace vexlib {
namespace hal {

// Abstract pneumatic actuator (single-channel solenoid)
class Pneumatic {
public:
  virtual ~Pneumatic() = default;
  virtual void set(bool extended) = 0;
  virtual bool get() const = 0;

  // Utility built-ins
  void toggle() { set(!get()); }

  // Fire for duration, then return to previous state.
  // Non-blocking: you call update() in your loop to time it out.
  void pulse_ms(int duration_ms) {
    pulse_target_ = true;
    pulse_end_ms_ = now_ms() + duration_ms;
    if (!pulsing_) {
      prev_state_ = get();
      set(true);
      pulsing_ = true;
    }
  }

  // Must be called periodically if you use pulse_ms().
  void update() {
    if (pulsing_ && now_ms() >= pulse_end_ms_) {
      set(prev_state_);
      pulsing_ = false;
      pulse_target_ = false;
    }
  }

protected:
  static int64_t now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  }

private:
  bool pulsing_{false};
  bool prev_state_{false};
  bool pulse_target_{false};
  int64_t pulse_end_ms_{0};
};

// Simple Pneumatic backed by function callbacks (great for adapters/mocks)
class PneumaticDO : public Pneumatic {
public:
  // write(true) energizes; read() returns cached/real line (optional).
  explicit PneumaticDO(std::function<void(bool)> write,
                       std::function<bool(void)> read = nullptr)
      : write_(std::move(write)), read_(std::move(read)) {}

  void set(bool extended) override {
    state_ = extended;
    write_(extended);
  }

  bool get() const override {
    if (read_) return read_();
    return state_;
  }

private:
  std::function<void(bool)> write_;
  std::function<bool(void)> read_;
  bool state_{false};
};

// Control many solenoids at once (e.g., two on a T)
class PneumaticGroup : public Pneumatic {
public:
  explicit PneumaticGroup(const std::vector<Pneumatic*>& chans) : chans_(chans) {}

  void set(bool extended) override {
    for (auto* p : chans_) p->set(extended);
    state_ = extended;
  }
  bool get() const override {
    // If any says true, report true (or change to all-of if you prefer)
    for (auto* p : chans_) if (p->get()) return true;
    return false;
  }

private:
  std::vector<Pneumatic*> chans_;
  bool state_{false};
};

} // namespace hal
} // namespace vexlib
