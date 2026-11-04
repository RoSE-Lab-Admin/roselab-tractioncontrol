// PI limiter on slip error that scales the commanded forward speed.
//
//   e      = slip error [-], positive when a wheel is above its limit (computed by the node)
//   e_pos  = max(e, 0)
//   I     += e * dt                 while e > 0
//   I     *= exp(-ki_bleed * dt)    while e <= 0 (bleeds toward 0, ki_bleed in 1/s)
//   k      = clamp(1 - kp * e_pos - ki * I, k_min, 1)
//
// Only the positive part of e enters the P term. With e < 0 the P term would push k above 1
// (clamped away) and the integral would be ignored, so recovery is set by the bleed rate.
// Anti-windup: I is clamped to [0, (1 - k_min) / ki], the value at which the I term alone
// reaches k_min.
// Monitor mode (enabled = false): k = 1 always and the integrator is held at 0.
// Non-finite e is treated as "no error information": the integrator bleeds.

#ifndef ROSEY_TRACTION__TRACTION_LIMITER_HPP_
#define ROSEY_TRACTION__TRACTION_LIMITER_HPP_

namespace rosey_traction
{

struct LimiterConfig
{
  double kp = 2.0;
  double ki = 0.5;
  double ki_bleed = 1.0;   // [1/s]
  double k_min = 0.2;
  bool enabled = false;
};

class TractionLimiter
{
public:
  explicit TractionLimiter(const LimiterConfig & config = LimiterConfig());

  // Advance by dt [s] with slip error e. Returns the new scale k in [k_min, 1].
  double update(double e, double dt);

  void reset();
  void setEnabled(bool enabled);

  double scale() const {return scale_;}
  double integral() const {return integral_;}
  const LimiterConfig & config() const {return config_;}

private:
  LimiterConfig config_;
  double integral_ = 0.0;
  double scale_ = 1.0;
};

}  // namespace rosey_traction

#endif  // ROSEY_TRACTION__TRACTION_LIMITER_HPP_
