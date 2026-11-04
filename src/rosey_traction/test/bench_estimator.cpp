// Times one 10 Hz cycle of the estimator: 4 wheels, cold start and warm start.
// Not a ctest. Run by hand: ./build/rosey_traction/bench_estimator

#include <chrono>
#include <cstdio>

#include "rosey_traction/slip_estimator.hpp"
#include "rosey_traction/slip_table_lookup.hpp"

int main()
{
  using clock = std::chrono::steady_clock;
  const rosey_traction::SlipEstimator est{rosey_traction::Params()};
  const double w = 39.3;
  const double t[4] = {2.14, 2.50, 2.86, 3.26};   // roughly s = 0.1 to 0.3 at 39.3 N
  const int cycles = 200;

  rosey_traction::SlipEstimate last[4];
  double sink = 0.0;

  auto t0 = clock::now();
  for (int c = 0; c < cycles; ++c) {
    for (int i = 0; i < 4; ++i) {
      last[i] = est.estimate(w, t[i]);
      sink += last[i].s + rosey_traction::sMax(w);
    }
  }
  const double cold_ms =
    std::chrono::duration<double, std::milli>(clock::now() - t0).count() / cycles;

  t0 = clock::now();
  for (int c = 0; c < cycles; ++c) {
    for (int i = 0; i < 4; ++i) {
      const rosey_traction::WarmStart warm{last[i].s, last[i].theta_f};
      last[i] = est.estimate(w * (1.0 + 0.002 * (c % 3)), t[i], warm);
      sink += last[i].s;
    }
  }
  const double warm_ms =
    std::chrono::duration<double, std::milli>(clock::now() - t0).count() / cycles;

  t0 = clock::now();
  for (int c = 0; c < 20; ++c) {
    for (int i = 0; i < 4; ++i) {sink += est.bisection(w, t[i]).s;}
  }
  const double fb_ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count() / 20;

  std::printf("4-wheel cycle, cold start:      %.3f ms\n", cold_ms);
  std::printf("4-wheel cycle, warm start:      %.3f ms\n", warm_ms);
  std::printf("4-wheel cycle, all fallback:    %.3f ms\n", fb_ms);
  std::printf("(checksum %.3f)\n", sink);
  return 0;
}
