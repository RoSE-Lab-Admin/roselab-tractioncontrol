// Motor current to wheel torque.
//
//   T = Kt * (|I| - I0) * sign(omega), and T = 0 when |I| <= I0.
//
// Kt is the wheel torque per amp [N m/A] (wheel torque/current constant, confirmed by the project
// manager 2026-11-04), so T is wheel torque directly and no gear ratio is applied.
// I0 is the no-load current [A]. sign(0) = 0, so a stopped wheel gives zero torque; the node
// skips the estimate for |omega| < omega_min anyway.

#ifndef ROSEY_TRACTION__CURRENT_TO_TORQUE_HPP_
#define ROSEY_TRACTION__CURRENT_TO_TORQUE_HPP_

#include <cmath>

namespace rosey_traction
{

inline double currentToTorque(double current_a, double omega, double kt, double i0)
{
  if (!std::isfinite(current_a) || !std::isfinite(omega)) {
    return 0.0;
  }
  const double excess = std::abs(current_a) - i0;
  if (excess <= 0.0) {
    return 0.0;
  }
  const double sign = (omega > 0.0) ? 1.0 : ((omega < 0.0) ? -1.0 : 0.0);
  return kt * excess * sign;
}

}  // namespace rosey_traction

#endif  // ROSEY_TRACTION__CURRENT_TO_TORQUE_HPP_
