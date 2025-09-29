#include "vex.h"
#include "vexlib/hal/vision.hpp"

using namespace vex;
using namespace vexlib::hal;

class V5VisionAdapter : public Vision {
public:
  bool capture() override {
    // Example: sensor.takeSnapshot(SIG1);
    objs.clear();
    objs.push_back({1, 158, 100, 30, 30, 0});
    return true;
  }
  const std::vector<VisionObject>& objects() const override { return objs; }
  std::optional<std::pair<int,int>> image_size_px() const override { return std::make_pair(316,212); }
private:
  std::vector<VisionObject> objs;
};

int main() {
  V5VisionAdapter cam;
  VisionPipeline pipe(cam);

  while (true) {
    pipe.update().filterSignature(1).filterMinArea(200).keepTopN(1);
    if (auto best = pipe.largest()) {
      Brain.Screen.printAt(10,40,"Object area=%d", best->area());
    }
    this_thread::sleep_for(50);
  }
}
