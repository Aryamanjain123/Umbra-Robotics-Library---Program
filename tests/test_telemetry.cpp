#include <gtest/gtest.h>
#include "vexlib/telemetry/telemetry.hpp"

using namespace vexlib::telemetry;

TEST(Telemetry, RingBufferSnapshot) {
  TelemetryBuffer buf(4);

  Sample s1{1.0, 1,2,3};
  Sample s2{2.0, 4,5,6};
  buf.push(s1);
  buf.push(s2);

  auto snap = buf.snapshot();
  ASSERT_EQ(snap.size(), 2u);
  EXPECT_NEAR(snap[0].x_m, 1.0, 1e-6);
  EXPECT_NEAR(snap[1].x_m, 4.0, 1e-6);
}
