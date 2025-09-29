#pragma once
#include <vector>
#include <cmath>
#include <cstddef>
#include <algorithm>
#include <numeric>

namespace vexlib {
namespace control {

// Simple motor feedforward model: u = kS*sign(v) + kV*v + kA*a
// Units of u are your actuator command (e.g., volts), v = velocity, a = acceleration.
class SimpleMotorFF {
public:
  SimpleMotorFF() = default;
  SimpleMotorFF(double kS, double kV, double kA, double sign_epsilon = 0.0)
    : kS_(kS), kV_(kV), kA_(kA), sign_eps_(std::max(0.0, sign_epsilon)) {}

  double kS() const { return kS_; }
  double kV() const { return kV_; }
  double kA() const { return kA_; }
  void setGains(double kS, double kV, double kA) { kS_ = kS; kV_ = kV; kA_ = kA; }
  void setSignEpsilon(double eps) { sign_eps_ = std::max(0.0, eps); }

  // Compute feedforward output for desired velocity & acceleration
  double compute(double vel, double accel) const {
    const double s = (sign_eps_ <= 0.0) ? ((vel>0)-(vel<0))
                                        : (vel == 0.0 ? 0.0 : vel / std::sqrt(vel*vel + sign_eps_*sign_eps_));
    return kS_ * s + kV_ * vel + kA_ * accel;
  }

  struct FitResult {
    double kS{0.0}, kV{0.0}, kA{0.0};
    double r2{0.0};
    double rmse{0.0};
    std::size_t n{0};
  };

  // Fit (kS, kV, kA) from samples using linear least squares.
  // Inputs are vectors of equal length:
  //   u[i] = commanded voltage (or pct), v[i] = velocity, a[i] = acceleration.
  // Uses sign(v) for kS column. Returns best-fit parameters and simple metrics.
  static FitResult fit(const std::vector<double>& u,
                       const std::vector<double>& v,
                       const std::vector<double>& a) {
    const std::size_t n = std::min({u.size(), v.size(), a.size()});
    FitResult out; out.n = n;
    if (n < 3) return out;

    // Build normal equations for X^T X and X^T y where X = [sign(v), v, a]
    double s11=0, s12=0, s13=0, s22=0, s23=0, s33=0;
    double t1=0,  t2=0,  t3=0;
    auto sgn = [](double x){ return (x>0)-(x<0); };

    for (std::size_t i=0;i<n;++i) {
      const double x1 = static_cast<double>(sgn(v[i]));
      const double x2 = v[i];
      const double x3 = a[i];
      const double yi = u[i];

      s11 += x1*x1; s12 += x1*x2; s13 += x1*x3;
      s22 += x2*x2; s23 += x2*x3; s33 += x3*x3;

      t1  += x1*yi;  t2  += x2*yi;  t3  += x3*yi;
    }

    // Symmetric matrix:
    // [s11 s12 s13][kS] = [t1]
    // [s12 s22 s23][kV]   [t2]
    // [s13 s23 s33][kA]   [t3]
    // Solve via Cramer's Rule (3x3) (sufficient here; numerically okay for well-conditioned data).
    const double det =
      s11*(s22*s33 - s23*s23) -
      s12*(s12*s33 - s13*s23) +
      s13*(s12*s23 - s13*s22);

    if (std::fabs(det) < 1e-12) {
      return out; // ill-conditioned; return zeros
    }

    const double det_kS =
      t1*(s22*s33 - s23*s23) -
      s12*(t2*s33 - s23*t3) +
      s13*(t2*s23 - s22*t3);

    const double det_kV =
      s11*(t2*s33 - s23*t3) -
      t1*(s12*s33 - s13*s23) +
      s13*(s12*t3 - t2*s13);

    const double det_kA =
      s11*(s22*t3 - t2*s23) -
      s12*(s12*t3 - t2*s13) +
      t1*(s12*s23 - s13*s22);

    out.kS = det_kS / det;
    out.kV = det_kV / det;
    out.kA = det_kA / det;

    // Compute simple metrics
    double y_mean = std::accumulate(u.begin(), u.begin()+n, 0.0) / static_cast<double>(n);
    double ss_tot = 0.0, ss_res = 0.0;
    for (std::size_t i=0;i<n;++i) {
      const double yhat = out.kS * sgn(v[i]) + out.kV * v[i] + out.kA * a[i];
      const double err  = u[i] - yhat;
      ss_res += err*err;
      const double dy = u[i] - y_mean;
      ss_tot += dy*dy;
    }
    out.rmse = std::sqrt(ss_res / static_cast<double>(n));
    out.r2   = (ss_tot <= 1e-12) ? 1.0 : (1.0 - ss_res / ss_tot);
    return out;
  }

private:
  double kS_{0.0}, kV_{0.0}, kA_{0.0};
  double sign_eps_{0.0};
};

} // namespace control
} // namespace vexlib
