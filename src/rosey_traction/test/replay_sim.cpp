// Print the slip-ramp replay trace (replay_scenario.hpp), for checking and tuning gains.
//
//   build/rosey_traction/replay_sim                    table every 0.5 s
//   build/rosey_traction/replay_sim --csv              every cycle, CSV
//   build/rosey_traction/replay_sim --kp=3 --ki=1 --ki_bleed=0.5 --k_min=0.3
//   build/rosey_traction/replay_sim --noise=0.02 --seed=7 --monitor

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "replay_scenario.hpp"

namespace
{
bool parse(const char * arg, const char * key, double & value)
{
  const size_t n = std::strlen(key);
  if (std::strncmp(arg, key, n) != 0 || arg[n] != '=') {return false;}
  value = std::atof(arg + n + 1);
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  auto config = replay::defaultConfig(true);
  replay::Options opt;
  bool csv = false;
  for (int i = 1; i < argc; ++i) {
    double seed = 0.0;
    if (std::strcmp(argv[i], "--csv") == 0) {csv = true; continue;}
    if (std::strcmp(argv[i], "--monitor") == 0) {config.limiter.enabled = false; continue;}
    if (parse(argv[i], "--kp", config.limiter.kp)) {continue;}
    if (parse(argv[i], "--ki", config.limiter.ki)) {continue;}
    if (parse(argv[i], "--ki_bleed", config.limiter.ki_bleed)) {continue;}
    if (parse(argv[i], "--k_min", config.limiter.k_min)) {continue;}
    if (parse(argv[i], "--noise", opt.noise)) {continue;}
    if (parse(argv[i], "--seed", seed)) {opt.seed = static_cast<uint32_t>(seed); continue;}
    std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
    return 2;
  }
  const auto err = config.validate();
  if (!err.empty()) {
    std::fprintf(stderr, "invalid config: %s\n", err.c_str());
    return 2;
  }

  const auto rows = replay::runSlipRamp(config, opt);
  if (csv) {
    std::printf("t,s_true,omega,current,s_est,s_max,error,integral,scale,limiting,status\n");
    for (const auto & r : rows) {
      std::printf(
        "%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%s\n", r.t, r.s_true, r.omega,
        r.current, r.s_est, r.s_max, r.error, r.integral, r.scale, r.limiting ? 1 : 0,
        r.status.c_str());
    }
    return 0;
  }

  std::printf(
    "kp=%.2f ki=%.2f ki_bleed=%.2f k_min=%.2f noise=%.3f seed=%u %s, s_max=%.3f\n",
    config.limiter.kp, config.limiter.ki, config.limiter.ki_bleed, config.limiter.k_min,
    opt.noise, opt.seed, config.limiter.enabled ? "enabled" : "monitor", rows.front().s_max);
  std::printf("   t  s_true  s_est   error  integ  scale  omega  bar\n");
  const int every = static_cast<int>(std::lround(0.5 * opt.rate_hz));
  for (size_t i = 0; i < rows.size(); i += every) {
    const auto & r = rows[i];
    const int bar = static_cast<int>(std::lround(r.scale * 20.0));
    std::printf(
      "%4.1f  %6.3f %6.3f %7.3f %6.3f %6.3f %6.3f  %s%s\n", r.t, r.s_true, r.s_est, r.error,
      r.integral, r.scale, r.omega, std::string(bar, '#').c_str(), r.limiting ? "  LIMIT" : "");
  }
  return 0;
}
