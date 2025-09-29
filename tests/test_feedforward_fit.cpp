#include <gtest/gtest.h>
#include "vexlib/control/feedforward.hpp"
#include <random>
#include <vector>
#include <cmath>

using vexlib::control::SimpleMotorFF;

TEST(FeedforwardFit, RecoversParametersFromNoisyData) {
  // Ground-truth parameters
  const double kS_true = 0.75;
  const double kV_true = 0.020;   // volts per (unit/s)
  const double kA_true = 0.004;   // volts per (unit/s^2)

  // Generate synthetic (u, v, a) with noise
  std::mt19937 rng(42);
  std::normal_distribution<double> n01(0.0, 0.05);  // output noise
  std::uniform_real_distribution<double> vdist(-100.0, 100.0);
  std::uniform_real_distribution<double> adist(-300.0, 300.0);

  const int N = 1200;
  std::vector<double> u, v, a;
  u.reserve(N); v.reserve(N); a.reserve(N);

  auto sgn = [](double x){ return (x>0)-(x<0); };

  for (int i = 0; i < N; ++i) {
    const double vi = vdist(rng);
    const double ai = adist(rng);
    const double ui = kS_true * sgn(vi) + kV_true * vi + kA_true * ai + n01(rng);
    v.push_back(vi);
    a.push_back(ai);
    u.push_back(ui);
  }

  auto fit = SimpleMotorFF::fit(u, v, a);

  EXPECT_EQ(fit.n, static_cast<size_t>(N));
  EXPECT_NEAR(fit.kS, kS_true, 0.10);   // within 0.10V
  EXPECT_NEAR(fit.kV, kV_true, 0.003);
  EXPECT_NEAR(fit.kA, kA_true, 0.0015);
  EXPECT_GT(fit.r2, 0.95);
}
