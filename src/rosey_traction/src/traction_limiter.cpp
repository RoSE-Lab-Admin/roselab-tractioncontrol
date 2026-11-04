#include "rosey_traction/traction_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace rosey_traction
{

TractionLimiter::TractionLimiter(const LimiterConfig & config)
: config_(config)
{
}

double TractionLimiter::update(double e, double dt)
{
  if (!config_.enabled) {
    reset();
    return scale_;
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    return scale_;
  }

  const bool have_error = std::isfinite(e) && e > 0.0;
  if (have_error) {
    integral_ += e * dt;
  } else {
    integral_ *= std::exp(-config_.ki_bleed * dt);
  }
  if (config_.ki > 0.0) {
    integral_ = std::clamp(integral_, 0.0, (1.0 - config_.k_min) / config_.ki);
  } else {
    integral_ = 0.0;
  }

  const double e_pos = have_error ? e : 0.0;
  const double k = 1.0 - config_.kp * e_pos - config_.ki * integral_;
  scale_ = std::clamp(k, config_.k_min, 1.0);
  return scale_;
}

void TractionLimiter::reset()
{
  integral_ = 0.0;
  scale_ = 1.0;
}

void TractionLimiter::setEnabled(bool enabled)
{
  if (enabled != config_.enabled) {
    reset();
  }
  config_.enabled = enabled;
}

}  // namespace rosey_traction
