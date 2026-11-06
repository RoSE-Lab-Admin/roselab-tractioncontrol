#include "rosey_traction/traction_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "rosey_traction/current_to_torque.hpp"
#include "rosey_traction/slip_table_lookup.hpp"

namespace rosey_traction
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

std::string CoreConfig::validate() const
{
  if (wheels.empty()) {return "no wheels configured";}
  for (const auto & w : wheels) {
    if (w.joint_name.empty()) {return "empty joint name";}
    if (!(w.kt > 0.0)) {return "kt must be > 0 for " + w.joint_name;}
    if (!(w.i0 >= 0.0)) {return "i0 must be >= 0 for " + w.joint_name;}
  }
  if (!(mass_kg > 0.0)) {return "mass_kg must be > 0";}
  if (!(gravity > 0.0)) {return "gravity must be > 0";}
  if (!(omega_min >= 0.0)) {return "omega_min must be >= 0";}
  if (!(turn_bypass_threshold >= 0.0)) {return "turn_bypass_threshold must be >= 0";}
  if (!(moving_cmd_min >= 0.0)) {return "moving_cmd_min must be >= 0";}
  if (frozen_cycles < 1) {return "frozen_cycles must be >= 1";}
  if (!(limiter.kp >= 0.0)) {return "kp must be >= 0";}
  if (!(limiter.ki > 0.0)) {return "ki must be > 0";}
  if (!(limiter.ki_bleed >= 0.0)) {return "ki_bleed must be >= 0";}
  if (!(limiter.k_min > 0.0 && limiter.k_min <= 1.0)) {return "k_min must be in (0, 1]";}
  return model.validate();
}

std::string CycleOutput::status() const
{
  if (warnings.empty()) {return "ok";}
  std::ostringstream os;
  for (size_t i = 0; i < warnings.size(); ++i) {
    if (i > 0) {os << "; ";}
    os << warnings[i];
  }
  return os.str();
}

TractionCore::TractionCore(const CoreConfig & config)
: config_(config),
  estimator_(config.model),
  limiter_(config.limiter),
  warm_(config.wheels.size()),
  last_inputs_(config.wheels.size())
{
}

void TractionCore::reset()
{
  limiter_.reset();
  std::fill(warm_.begin(), warm_.end(), std::nullopt);
  std::fill(last_inputs_.begin(), last_inputs_.end(), WheelInput());
  frozen_count_ = 0;
}

void TractionCore::setEnabled(bool enabled)
{
  config_.limiter.enabled = enabled;
  limiter_.setEnabled(enabled);
}

bool TractionCore::updateFrozen(const CycleInput & in)
{
  if (!in.joints_new) {
    return frozen_count_ >= config_.frozen_cycles;
  }
  // Moving: a wheel turns, or a forward/backward command is active. The second case catches
  // telemetry that froze at zero while the rover was told to drive.
  bool moving = in.have_command && std::abs(in.cmd_linear_x) > config_.moving_cmd_min;
  bool identical = true;
  for (size_t i = 0; i < in.wheels.size(); ++i) {
    const auto & now = in.wheels[i];
    const auto & last = last_inputs_[i];
    if (std::abs(now.omega) >= config_.omega_min) {moving = true;}
    // Exact comparison on purpose: live telemetry always jitters in the last digits.
    // NaN compares equal to NaN here, so a stuck NaN also counts as frozen.
    const auto same = [](double a, double b) {
        return a == b || (std::isnan(a) && std::isnan(b));
      };
    if (now.present != last.present || !same(now.omega, last.omega) ||
      !same(now.current, last.current))
    {
      identical = false;
    }
  }
  last_inputs_ = in.wheels;
  frozen_count_ = (moving && identical) ? frozen_count_ + 1 : 0;
  return frozen_count_ >= config_.frozen_cycles;
}

CycleOutput TractionCore::step(const CycleInput & in)
{
  CycleOutput out;
  const size_t n = config_.wheels.size();
  out.wheels.resize(n);

  // Telemetry checks.
  bool telemetry_ok = in.joints_fresh && in.wheels.size() == n;
  if (!in.joints_fresh) {
    out.warnings.push_back("joint states stale");
  } else if (in.wheels.size() != n) {
    out.warnings.push_back("internal: wheel input count mismatch");
  } else {
    for (size_t i = 0; i < n; ++i) {
      if (!in.wheels[i].present) {
        telemetry_ok = false;
        out.warnings.push_back(config_.wheels[i].joint_name + " missing velocity or current");
      }
    }
  }
  bool frozen = false;
  if (telemetry_ok) {
    frozen = updateFrozen(in);
    if (frozen) {
      telemetry_ok = false;
      out.warnings.push_back("joint states frozen");
    }
  }

  // Load.
  double tilt_cos = 1.0;
  if (in.imu_fresh && std::isfinite(in.tilt_cos) && std::isfinite(in.pitch)) {
    tilt_cos = in.tilt_cos;
    out.pitch = in.pitch;
  } else {
    out.warnings.push_back("imu stale, pitch = 0");
  }
  const double load = config_.mass_kg * config_.gravity * tilt_cos / static_cast<double>(n);

  // Per-wheel estimate.
  double error = kNaN;
  bool any_reverse = false;
  for (size_t i = 0; i < n; ++i) {
    const auto & cfg = config_.wheels[i];
    auto & w = out.wheels[i];
    w.joint_name = cfg.joint_name;
    w.load = load;
    w.slip_est = kNaN;
    w.theta_f = kNaN;
    w.s_max = sMax(load);
    const auto t_max = (load > 0.0) ? torqueAt(load, w.s_max, config_.model) : std::nullopt;
    w.t_max = t_max ? *t_max : kNaN;

    if (i < in.wheels.size()) {
      w.omega = in.wheels[i].omega;
      w.current = in.wheels[i].current;
      w.torque = currentToTorque(w.current, w.omega, cfg.kt, cfg.i0);
    }

    const bool torque_usable = telemetry_ok && load > 0.0 && std::isfinite(w.omega) &&
      std::abs(w.omega) >= config_.omega_min && std::abs(w.current) > cfg.i0;
    if (torque_usable && w.omega < 0.0) {
      any_reverse = true;
    }
    if (!torque_usable || w.omega < 0.0) {
      warm_[i].reset();
      continue;
    }

    // Torque term first: it is available even when the estimator fails, which is its main job
    // (see the header). With a valid estimate it agrees in sign with the slip term.
    double e_i = kNaN;
    if (std::isfinite(w.t_max) && w.t_max > 0.0) {
      e_i = (w.torque - w.t_max) / w.t_max * w.s_max;
    }

    const auto est = estimator_.estimate(load, w.torque, warm_[i]);
    if (est.valid) {
      warm_[i] = WarmStart{est.s, est.theta_f};
      w.estimate_valid = true;
      w.slip_est = est.s;
      w.theta_f = est.theta_f;
      const double e_slip = w.slip_est - w.s_max;
      e_i = std::isnan(e_i) ? e_slip : std::max(e_i, e_slip);
    } else {
      warm_[i].reset();
    }

    if (!std::isnan(e_i)) {
      w.limiting = e_i > 0.0;
      error = std::isnan(error) ? e_i : std::max(error, e_i);
    }
  }
  if (any_reverse) {
    out.warnings.push_back("reversing, no estimate");
  }
  out.error = error;

  // Scale.
  if (!telemetry_ok) {
    limiter_.reset();
    out.scale = 1.0;
  } else if (in.have_command && std::abs(in.cmd_angular_z) > config_.turn_bypass_threshold) {
    limiter_.reset();
    out.scale = 1.0;
    out.bypass_turning = true;
  } else {
    out.scale = limiter_.update(error, in.dt);
  }
  return out;
}

}  // namespace rosey_traction
