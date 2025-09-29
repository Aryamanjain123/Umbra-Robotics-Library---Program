#include <gtest/gtest.h>
#include "vexlib/hal/vision.hpp"
using namespace vexlib::hal;

// Minimal fake vision for tests
class FakeVision : public Vision {
public:
  bool capture() override { return true; }
  const std::vector<VisionObject>& objects() const override { return objs; }
  std::optional<std::pair<int,int>> image_size_px() const override { return std::make_pair(316,212); }
  std::vector<VisionObject> objs;
};

TEST(Vision, PipelineFilters) {
  FakeVision v;
  v.objs = {
    {1, 100,100, 20,20, 0},
    {1, 150,100, 40,40, 0},
    {2,  50, 80, 15,10, 0}
  };

  VisionPipeline pipe(v);
  pipe.update().filterSignature(1).filterMinArea(400).keepTopN(1);

  auto top = pipe.largest();
  ASSERT_TRUE(top.has_value());
  EXPECT_EQ(top->width, 40);
  EXPECT_EQ(top->height, 40);
}

TEST(Vision, EstimateYaw) {
  FakeVision v;
  VisionObject o{1, 158, 100, 30,30, 0}; // roughly center in 316px
  double yaw = VisionPipeline::estimateYawDeg(o, 316, 60.0);
  EXPECT_NEAR(yaw, 0.0, 2.0); // within a couple degrees
}
