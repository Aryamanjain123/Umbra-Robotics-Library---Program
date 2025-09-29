#pragma once
#if __has_include("pros/distance.hpp")
  #include "pros/distance.hpp"
  #define VEXLIB_HAS_PROS 1
#else
  #define VEXLIB_HAS_PROS 0
#endif

namespace vexlib::hal {

class Distance {
 public:
  explicit Distance(int port): port_(port)
#if VEXLIB_HAS_PROS
    , dist_(port)
#endif
  {}
  double mm() const {
#if VEXLIB_HAS_PROS
    return dist_.get();
#else
    return 0.0;
#endif
  }

 private:
  int port_;
#if VEXLIB_HAS_PROS
  pros::Distance dist_;
#endif
};

} // namespace vexlib::hal
