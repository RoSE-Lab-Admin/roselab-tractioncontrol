// Per-cycle traction logic, without ROS. traction_node gathers the inputs from topics and
// calls TractionCore::step once per timer tick; the replay harness (test/replay_scenario.hpp)
// calls it directly.
//
// Per wheel (wheels are matched to telemetry by joint name in the node, never by index):
//   T   = Kt * (|I| - I0) * sign(omega)
//   W   = m * g * tilt_cos / n_wheels                         (equal split for now)
//   (theta_f, s) = SlipEstimator::estimate(W, T), warm-started from the wheel's last valid result
//   s_max = sMax(W),  t_max = torqueAt(W, s_max)
// No estimate this cycle (estimate_valid = false) when:
//   - telemetry is stale, missing or frozen
//   - |omega| < omega_min
//   - omega < 0: reversing. The current sign convention is not confirmed yet (bench test,
//     Phase 4), and the model is only validated for driving torque.
//   - |I| <= I0: no driving torque (mock hardware reports current = 0 here)
//   - the estimator fails
//
// Slip error, per wheel with a usable torque (forward, |omega| >= omega_min, |I| > I0), in slip units:
//   torque term  e_T = (T_i - t_max_i) / t_max_i * s_max_i
//   slip term    e_s = s_i - s_max_i                     (only if the estimate is valid)
//   e_i = max(e_T, e_s), or e_T alone when the estimate is invalid
// The torque term is the fractional torque excess over t_max scaled by s_max, so being 10%
// over t_max at s_max = 0.2 counts like being 0.02 over s_max.
// The torque term mainly backs up an invalid estimate. T rises with s, so with a valid estimate
// T > t_max and s > s_max happen together and the two terms agree in sign. The estimator fails
// most often at very high slip (T above T(W, 0.9)), and there the torque term still reports the
// excess instead of going silent.
// e = max over wheels, NaN when no wheel has a usable torque (the limiter then bleeds its
// integrator).
//
// Frozen telemetry: on a serial timeout the hardware read() keeps its old values, so the
// broadcaster keeps publishing the same numbers. If, while the rover is moving (some wheel
// has |omega| >= omega_min, or the command has |linear.x| > moving_cmd_min), frozen_cycles
// consecutive new messages carry exactly the same velocity and current for every wheel, the
// telemetry is treated as frozen.
//
// Scale k:
//   - telemetry stale or frozen: k = 1, limiter reset
//   - |angular.z| > turn_bypass_threshold: k = 1, limiter reset, bypass_turning = true
//   - otherwise k = TractionLimiter::update(e, dt); with enabled = false that is always 1

#ifndef ROSEY_TRACTION__TRACTION_CORE_HPP_
#define ROSEY_TRACTION__TRACTION_CORE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rosey_traction/slip_estimator.hpp"
#include "rosey_traction/terramechanics.hpp"
#include "rosey_traction/traction_limiter.hpp"

namespace rosey_traction
{

struct WheelConfig
{
  std::string joint_name;
  double kt = 1.77;    // wheel torque per amp [N m/A]
  double i0 = 0.15;    // no-load current [A]
};

struct CoreConfig
{
  std::vector<WheelConfig> wheels;
  double mass_kg = 16.01;
  double gravity = 9.81;
  double omega_min = 0.05;              // [rad/s]
  double turn_bypass_threshold = 0.05;  // [rad/s]
  double moving_cmd_min = 0.01;         // [m/s], command above this counts as moving (frozen guard)
  int frozen_cycles = 10;
  LimiterConfig limiter;
  Params model;

  // Empty string if valid, otherwise a description of the first problem found.
  std::string validate() const;
};

struct WheelInput
{
  bool present = false;  // joint (with velocity and current) found in the latest telemetry
  double omega = 0.0;    // [rad/s]
  double current = 0.0;  // [A]
};

struct CycleInput
{
  std::vector<WheelInput> wheels;  // same order as CoreConfig::wheels
  bool joints_fresh = false;       // telemetry received within stale_timeout
  bool joints_new = false;         // a new telemetry message arrived since the last step
  bool imu_fresh = false;          // IMU received within its timeout
  double tilt_cos = 1.0;           // body z axis . world z axis, from the IMU
  double pitch = 0.0;              // [rad], reported only
  bool have_command = false;       // a /cmd_vel arrived within cmd_timeout
  double cmd_linear_x = 0.0;       // [m/s]
  double cmd_angular_z = 0.0;      // [rad/s]
  double dt = 0.0;                 // time since the last step [s]
};

struct WheelOutput
{
  std::string joint_name;
  double omega = 0.0;
  double current = 0.0;
  double torque = 0.0;
  double load = 0.0;
  double slip_est;      // NaN when !estimate_valid
  double theta_f;       // NaN when !estimate_valid
  double s_max;
  double t_max;         // NaN when the model has no solution at this load
  bool estimate_valid = false;
  bool limiting = false;   // this wheel's error e_i > 0
};

struct CycleOutput
{
  std::vector<WheelOutput> wheels;
  double pitch = 0.0;              // 0 when the IMU is stale
  double error = 0.0;              // slip error e fed to the limiter (NaN if none)
  double scale = 1.0;
  bool bypass_turning = false;
  std::vector<std::string> warnings;

  // "ok" or the warnings joined with "; ".
  std::string status() const;
};

class TractionCore
{
public:
  explicit TractionCore(const CoreConfig & config);

  CycleOutput step(const CycleInput & in);

  void reset();
  void setEnabled(bool enabled);
  const CoreConfig & config() const {return config_;}
  const TractionLimiter & limiter() const {return limiter_;}

private:
  bool updateFrozen(const CycleInput & in);

  CoreConfig config_;
  SlipEstimator estimator_;
  TractionLimiter limiter_;
  std::vector<std::optional<WarmStart>> warm_;
  std::vector<WheelInput> last_inputs_;
  int frozen_count_ = 0;
};

}  // namespace rosey_traction

#endif  // ROSEY_TRACTION__TRACTION_CORE_HPP_
