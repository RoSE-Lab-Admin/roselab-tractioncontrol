// Onboard slip estimator: recover (theta_f, s) for one wheel from its load W and torque T.
//
// Port of estimateSlip in docs/traction/reference/rosey_terramechanics_octave.m:
// Newton-Raphson on W(thf, s) = W_m and T(thf, s) = T_m, finite-difference Jacobian,
// Cramer's rule, step limits, clamping, singular-Jacobian guard, 20 iterations.
//
// Fallback (not in the Octave script): if Newton does not converge, bisection on s in [0, 0.9]
// using T(s) at the given W, as slipFromTorque in rosey_current_tradestudy.m. The fallback
// result is valid only if T_m lies between T(W, 0) and T(W, 0.9).
//
// Never throws. Bad input (non-finite, W <= 0, T < 0, W beyond the model range) gives
// valid = false.

#ifndef ROSEY_TRACTION__SLIP_ESTIMATOR_HPP_
#define ROSEY_TRACTION__SLIP_ESTIMATOR_HPP_

#include <optional>

#include "rosey_traction/terramechanics.hpp"

namespace rosey_traction
{

struct SlipEstimate
{
  double s = 0.0;            // estimated slip [-]
  double theta_f = 0.0;      // estimated entry angle [rad]
  bool valid = false;
  int iterations = 0;        // Newton iterations used (Octave "it")
  bool used_fallback = false;
};

struct WarmStart
{
  double s = 0.0;
  double theta_f = 0.0;
};

class SlipEstimator
{
public:
  explicit SlipEstimator(const Params & params);

  // Estimate slip from wheel load w [N] and driving torque t [N m].
  // warm: last cycle's result. Without it the initial guess is s0 = 0.1 and
  // thf0 = solveEntryAngle(w, 0.1), as in the Octave demo.
  SlipEstimate estimate(double w, double t, const std::optional<WarmStart> & warm = std::nullopt) const;

  // Newton-Raphson only (exact port of estimateSlip). Exposed for testing.
  SlipEstimate newton(double w, double t, double s0, double thf0) const;

  // Bisection on s at fixed load (slipFromTorque). Exposed for testing.
  SlipEstimate bisection(double w, double t) const;

  const Params & params() const {return params_;}

  static constexpr int kMaxIterations = 20;
  static constexpr double kSMaxClamp = 0.95;        // Newton clamp on s
  static constexpr double kFallbackSHigh = 0.9;     // bisection bracket upper end
  static constexpr int kFallbackIterations = 40;
  static constexpr double kFallbackThfTol = 1e-8;   // entry angle tolerance inside the fallback

private:
  Params params_;
};

}  // namespace rosey_traction

#endif  // ROSEY_TRACTION__SLIP_ESTIMATOR_HPP_
