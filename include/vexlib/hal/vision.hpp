#pragma once
#include <cstdint>
#include <vector>
#include <optional>
#include <algorithm>
#include <cmath>

namespace vexlib {
namespace hal {

// Unified object detection structure
struct VisionObject {
  int signature_id{0};   // user-defined signature/class
  int x{0};              // bbox center x (pixels)
  int y{0};              // bbox center y (pixels)
  int width{0};          // bbox width (pixels)
  int height{0};         // bbox height (pixels)
  int angle_deg{0};      // object orientation (deg) if available
  int area() const { return width * height; }
};

// Platform-agnostic Vision interface (implement for your hardware)
class Vision {
public:
  virtual ~Vision() = default;

  // Capture/refresh sensor data; return true if new frame
  virtual bool capture() = 0;

  // Return latest objects (after capture())
  virtual const std::vector<VisionObject>& objects() const = 0;

  // Optional: image size for normalization
  virtual std::optional<std::pair<int,int>> image_size_px() const { return std::nullopt; }
};

// ---------------- Pipeline utilities (filtering, selection) ----------------

class VisionPipeline {
public:
  explicit VisionPipeline(Vision& sensor) : sensor_(sensor) {}

  // Pull a frame (returns number of objects)
  int update() {
    sensor_.capture();
    cache_ = sensor_.objects();  // make a copy so we can filter
    return static_cast<int>(cache_.size());
  }

  // Keep only objects matching a signature (if sig==-1, keep all)
  VisionPipeline& filterSignature(int sig) {
    if (sig < 0) return *this;
    std::vector<VisionObject> out;
    out.reserve(cache_.size());
    for (auto& o : cache_) if (o.signature_id == sig) out.push_back(o);
    cache_.swap(out);
    return *this;
  }

  // Remove small objects by bbox area
  VisionPipeline& filterMinArea(int min_area_px) {
    std::vector<VisionObject> out;
    out.reserve(cache_.size());
    for (auto& o : cache_) if (o.area() >= min_area_px) out.push_back(o);
    cache_.swap(out);
    return *this;
  }

  // Keep N largest by area
  VisionPipeline& keepTopN(int n) {
    if (n <= 0) { cache_.clear(); return *this; }
    std::sort(cache_.begin(), cache_.end(),
              [](const VisionObject& a, const VisionObject& b){ return a.area() > b.area(); });
    if ((int)cache_.size() > n) cache_.resize(n);
    return *this;
  }

  // Return largest (after filters), or nullopt
  std::optional<VisionObject> largest() const {
    if (cache_.empty()) return std::nullopt;
    const VisionObject* best = &cache_.front();
    int bestA = best->area();
    for (auto& o : cache_) {
      const int A = o.area();
      if (A > bestA) { bestA = A; best = &o; }
    }
    return *best;
  }

  // All filtered objects
  const std::vector<VisionObject>& results() const { return cache_; }

  // Utility: estimate horizontal angular offset from image center
  // Requires image width (px) and camera horizontal FOV (deg)
  static double estimateYawDeg(const VisionObject& o, int img_w_px, double hfov_deg) {
    const double norm = (2.0 * (double)o.x / (double)img_w_px) - 1.0; // -1..+1
    return (hfov_deg * 0.5) * norm;
  }

private:
  Vision& sensor_;
  std::vector<VisionObject> cache_;
};

} // namespace hal
} // namespace vexlib
