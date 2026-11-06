// Offline replay of synthetic telemetry through TractionCore (the node's per-cycle logic).
// Used by the software_traction_replay gtest and the replay_sim tool.
//
// Slip ramp scenario, all four wheels alike, flat ground, IMU fresh, constant command:
//   0-2 s    s = 0.1
//   2-7 s    s ramps 0.1 -> 0.6
//   7-8 s    s = 0.6
//   8-13 s   s ramps 0.6 -> 0.1
//   13-18 s  s = 0.1
// Each cycle the motor current is the one that gives the model torque at the true slip,
// I = T(W, s) / Kt + I0, times (1 + noise * randn) with a seeded generator.
// The wheel speed follows the last scaled command, omega = k * v_cmd / r_wheel, so the trace
// shows the wheels slowing when the limiter acts. Slip itself is prescribed: this checks the
// controller logic, not the rover dynamics.

#ifndef ROSEY_TRACTION__TEST__REPLAY_SCENARIO_HPP_
#define ROSEY_TRACTION__TEST__REPLAY_SCENARIO_HPP_

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "rosey_traction/traction_core.hpp"

namespace replay
{

struct Options
{
  double rate_hz = 10.0;
  double cmd_x = 0.2;           // commanded linear.x [m/s]
  double wheel_radius = 0.150;  // diff_drive wheel_radius, for omega from the command [m]
  double noise = 0.01;          // relative current noise (1 sigma)
  uint32_t seed = 1;
};

struct Row
{
  double t;
  double s_true;
  double omega;
  double current;
  double s_est;      // front-left wheel, NaN if invalid
  double s_max;
  double error;
  double integral;
  double scale;
  bool limiting;
  std::string status;
};

inline double slipProfile(double t)
{
  if (t < 2.0) {return 0.1;}
  if (t < 7.0) {return 0.1 + 0.5 * (t - 2.0) / 5.0;}
  if (t < 8.0) {return 0.6;}
  if (t < 13.0) {return 0.6 - 0.5 * (t - 8.0) / 5.0;}
  return 0.1;
}

constexpr double kDuration = 18.0;

// Seeded standard normal (Box-Muller on raw mt19937 output, same on every platform).
class Gaussian
{
public:
  explicit Gaussian(uint32_t seed)
  : gen_(seed) {}
  double operator()()
  {
    const double u1 = (static_cast<double>(gen_()) + 0.5) / 4294967296.0;
    const double u2 = (static_cast<double>(gen_()) + 0.5) / 4294967296.0;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * rosey_traction::kPi * u2);
  }

private:
  std::mt19937 gen_;
};

inline rosey_traction::CoreConfig defaultConfig(bool enabled)
{
  rosey_traction::CoreConfig c;
  for (const char * name : {"front_left_wheel_joint", "front_right_wheel_joint",
      "back_left_wheel_joint", "back_right_wheel_joint"})
  {
    c.wheels.push_back({name, 1.77, 0.15});
  }
  c.limiter.enabled = enabled;
  return c;
}

inline std::vector<Row> runSlipRamp(
  const rosey_traction::CoreConfig & config, const Options & opt = Options())
{
  rosey_traction::TractionCore core(config);
  Gaussian randn(opt.seed);
  const double dt = 1.0 / opt.rate_hz;
  const size_t n = config.wheels.size();
  const double load = config.mass_kg * config.gravity / static_cast<double>(n);
  const int steps = static_cast<int>(std::lround(kDuration * opt.rate_hz));

  std::vector<Row> rows;
  rows.reserve(steps);
  double scale = 1.0;
  for (int k = 0; k < steps; ++k) {
    const double t = k * dt;
    const double s_true = slipProfile(t);
    const double omega = scale * opt.cmd_x / opt.wheel_radius;
    const auto torque = rosey_traction::torqueAt(load, s_true, config.model);

    rosey_traction::CycleInput in;
    in.wheels.resize(n);
    double current0 = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const auto & w = config.wheels[i];
      const double clean = (torque ? *torque : 0.0) / w.kt + w.i0;
      in.wheels[i] = {true, omega, clean * (1.0 + opt.noise * randn())};
      if (i == 0) {current0 = in.wheels[i].current;}
    }
    in.joints_fresh = true;
    in.joints_new = true;
    in.imu_fresh = true;
    in.tilt_cos = 1.0;
    in.pitch = 0.0;
    in.have_command = true;
    in.cmd_linear_x = opt.cmd_x;
    in.cmd_angular_z = 0.0;
    in.dt = (k == 0) ? 0.0 : dt;

    const auto out = core.step(in);
    scale = out.scale;
    rows.push_back(Row{
        t, s_true, omega, current0, out.wheels[0].slip_est, out.wheels[0].s_max, out.error,
        core.limiter().integral(), out.scale, out.wheels[0].limiting, out.status()});
  }
  return rows;
}

}  // namespace replay

#endif  // ROSEY_TRACTION__TEST__REPLAY_SCENARIO_HPP_
