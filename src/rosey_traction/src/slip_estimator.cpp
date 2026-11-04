#include "rosey_traction/slip_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace rosey_traction
{

SlipEstimator::SlipEstimator(const Params & params)
: params_(params)
{
}

SlipEstimate SlipEstimator::estimate(
  double w, double t, const std::optional<WarmStart> & warm) const
{
  SlipEstimate out;
  if (!std::isfinite(w) || !std::isfinite(t) || w <= 0.0 || t < 0.0) {
    return out;   // valid = false
  }

  double s0 = 0.1;
  double thf0 = 0.0;
  if (warm && std::isfinite(warm->s) && std::isfinite(warm->theta_f)) {
    s0 = std::clamp(warm->s, 0.0, kSMaxClamp);
    thf0 = std::clamp(warm->theta_f, 1e-4, params_.thf_max);
  } else {
    const auto thf = solveEntryAngle(w, s0, params_);
    if (!thf) {
      return out;   // load beyond model range
    }
    thf0 = *thf;
  }

  out = newton(w, t, s0, thf0);
  if (out.valid) {
    return out;
  }
  SlipEstimate fb = bisection(w, t);
  fb.iterations = out.iterations;
  return fb;
}

// estimateSlip
SlipEstimate SlipEstimator::newton(double w_m, double t_m, double s0, double thf0) const
{
  const Params & p = params_;
  double x0 = thf0;   // theta_f
  double x1 = s0;     // s
  bool ok = false;
  const double tol_w = 1e-3 * std::max(std::abs(w_m), 1.0);
  const double tol_t = 1e-3 * std::max(std::abs(t_m), 1e-3);
  const double d0 = 1e-5;                 // finite-difference step [rad]
  const double d1 = 1e-4;                 // finite-difference step [-]
  const double max_step0 = 5.0 * kDegToRad;
  const double max_step1 = 0.1;

  int it = 1;
  for (; it <= kMaxIterations; ++it) {
    const Forces f = wheelForces(x0, x1, p);
    const double r0 = f.W - w_m;
    const double r1 = f.T - t_m;
    if (std::abs(r0) < tol_w && std::abs(r1) < tol_t) {
      ok = true;
      break;
    }
    const Forces f1 = wheelForces(x0 + d0, x1, p);
    const Forces f2 = wheelForces(x0, x1 + d1, p);
    const double j00 = (f1.W - f.W) / d0, j01 = (f2.W - f.W) / d1;
    const double j10 = (f1.T - f.T) / d0, j11 = (f2.T - f.T) / d1;
    const double det = j00 * j11 - j01 * j10;
    const double n_col0 = std::hypot(j00, j10);
    const double n_col1 = std::hypot(j01, j11);
    if (!std::isfinite(det) || std::abs(det) < 1e-10 * n_col0 * n_col1) {
      break;   // singular: hold last estimate (Octave), caller falls back
    }
    // Cramer's rule: dx = -inv(J) * R
    double dx0 = -(j11 * r0 - j01 * r1) / det;
    double dx1 = -(-j10 * r0 + j00 * r1) / det;
    dx0 = std::clamp(dx0, -max_step0, max_step0);
    dx1 = std::clamp(dx1, -max_step1, max_step1);
    x0 = std::clamp(x0 + dx0, 1e-4, p.thf_max);
    x1 = std::clamp(x1 + dx1, 0.0, kSMaxClamp);
  }

  SlipEstimate out;
  out.theta_f = x0;
  out.s = x1;
  out.valid = ok;
  out.iterations = std::min(it, kMaxIterations);
  return out;
}

// slipFromTorque
SlipEstimate SlipEstimator::bisection(double w, double t_m) const
{
  SlipEstimate out;
  out.used_fallback = true;
  if (!std::isfinite(w) || !std::isfinite(t_m) || w <= 0.0 || t_m < 0.0) {
    return out;
  }
  const auto t_lo = torqueAt(w, 0.0, params_, kFallbackThfTol);
  const auto t_hi = torqueAt(w, kFallbackSHigh, params_, kFallbackThfTol);
  if (!t_lo || !t_hi || t_m < *t_lo || t_m > *t_hi) {
    return out;   // load out of range, or torque not bracketed
  }

  double lo = 0.0;
  double hi = kFallbackSHigh;
  for (int it = 0; it < kFallbackIterations; ++it) {
    const double mid = 0.5 * (lo + hi);
    const auto t_mid = torqueAt(w, mid, params_, kFallbackThfTol);
    if (!t_mid) {
      return out;
    }
    if (*t_mid > t_m) {hi = mid;} else {lo = mid;}
  }
  const double s = 0.5 * (lo + hi);
  const auto thf = solveEntryAngle(w, s, params_);
  if (!thf) {
    return out;
  }
  out.s = s;
  out.theta_f = *thf;
  out.valid = true;
  return out;
}

}  // namespace rosey_traction
