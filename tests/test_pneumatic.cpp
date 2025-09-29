#include <gtest/gtest.h>
#include "vexlib/hal/pneumatic.hpp"
using namespace vexlib::hal;

TEST(Pneumatic, ToggleAndPulse) {
  bool line = false;
  PneumaticDO p(
    [&](bool v){ line = v; },
    [&](){ return line; }
  );

  EXPECT_FALSE(p.get());
  p.set(true);
  EXPECT_TRUE(p.get());
  p.toggle();
  EXPECT_FALSE(p.get());

  // Pulse non-blocking
  p.pulse_ms(10);
  EXPECT_TRUE(p.get());
  // simulate time passing
  for (int i=0;i<20;++i) { p.update(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  p.update();
  EXPECT_FALSE(p.get());
}

TEST(Pneumatic, GroupSetsAll) {
  bool a=false,b=false;
  PneumaticDO pa([&](bool v){a=v;}, [&](){return a;});
  PneumaticDO pb([&](bool v){b=v;}, [&](){return b;});
  PneumaticGroup g({&pa, &pb});
  g.set(true);
  EXPECT_TRUE(a && b && g.get());
}
