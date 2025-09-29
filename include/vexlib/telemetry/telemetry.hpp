#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <mutex>
#include <optional>
#include "vexlib/utils/ring_buffer.hpp"

namespace vexlib {
namespace telemetry {

struct Sample {
  double t_sec{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double theta_rad{0.0};
  double target_x_m{0.0};
  double target_y_m{0.0};
  double target_theta_rad{0.0};
  double error{0.0};
  double output{0.0};
};

// Thread-safe ring buffer for real-time sampling
class TelemetryBuffer {
 public:
  explicit TelemetryBuffer(size_t capacity = 512) : buffer_(capacity) {}

  void push(const Sample& s) {
    std::lock_guard<std::mutex> lock(mu_);
    buffer_.push(s);
  }

  std::vector<Sample> snapshot() const {
    std::lock_guard<std::mutex> lock(mu_);
    return buffer_.to_vector();
  }

 private:
  mutable std::mutex mu_;
  utils::RingBuffer<Sample> buffer_;
};

// SD-card CSV logger
class CSVLogger {
 public:
  CSVLogger() = default;

  bool open(const std::string& filename) {
    file_.open(filename);
    if (!file_.is_open()) return false;
    file_ << "t,x,y,theta,target_x,target_y,target_theta,error,output\n";
    return true;
  }

  void log(const Sample& s) {
    if (!file_.is_open()) return;
    file_ << s.t_sec << ","
          << s.x_m << "," << s.y_m << "," << s.theta_rad << ","
          << s.target_x_m << "," << s.target_y_m << "," << s.target_theta_rad << ","
          << s.error << "," << s.output << "\n";
  }

  void close() {
    if (file_.is_open()) file_.close();
  }

 private:
  std::ofstream file_;
};

}  // namespace telemetry
}  // namespace vexlib
