// Phase 3 replay: a slip ramp from 0.1 to 0.6 and back at constant command, through
// TractionCore. k must stay at 1 below s_max, drop once slip passes s_max, never go below
// k_min, and recover to 1 after slip falls back. See replay_scenario.hpp for the scenario.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "replay_scenario.hpp"

namespace
{
constexpr double kMargin = 0.03;  // slip margin around s_max for 1% current noise
}

TEST(TractionReplay, SlipRampLimitsAndRecovers)
{
  const auto config = replay::defaultConfig(true);
  for (uint32_t seed = 1; seed <= 5; ++seed) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    replay::Options opt;
    opt.seed = seed;
    const auto rows = replay::runSlipRamp(config, opt);
    ASSERT_FALSE(rows.empty());
    const double s_max = rows.front().s_max;

    bool dropped = false;
    double last_recovery_scale = 0.0;
    for (const auto & r : rows) {
      SCOPED_TRACE("t=" + std::to_string(r.t));
      EXPECT_EQ(r.status, "ok");
      EXPECT_GE(r.scale, config.limiter.k_min);
      EXPECT_LE(r.scale, 1.0);

      // Up-ramp, clearly below s_max: untouched.
      if (r.t < 7.0 && r.s_true < s_max - kMargin) {
        EXPECT_DOUBLE_EQ(r.scale, 1.0);
      }
      // The first reduction happens only once slip has reached s_max.
      if (!dropped && r.scale < 1.0) {
        dropped = true;
        EXPECT_GE(r.s_true, s_max - kMargin);
        EXPECT_LE(r.t, 7.0);
      }
      // Holding at s = 0.6: clearly limiting.
      if (r.t >= 7.0 && r.t < 8.0) {
        EXPECT_TRUE(r.limiting);
        EXPECT_LT(r.scale, 0.6);
      }
      // Down-ramp, clearly below s_max: only recovering (integrator bleeds).
      if (r.t >= 8.0 && r.s_true < s_max - kMargin) {
        EXPECT_GE(r.scale, last_recovery_scale - 1e-12);
        last_recovery_scale = r.scale;
      }
    }
    EXPECT_TRUE(dropped);
    // Recovered by the end of the 5 s low-slip hold.
    EXPECT_GT(rows.back().scale, 0.99);
  }
}

TEST(TractionReplay, MonitorModeOnlyReports)
{
  const auto rows = replay::runSlipRamp(replay::defaultConfig(false));
  bool any_limiting = false;
  for (const auto & r : rows) {
    EXPECT_DOUBLE_EQ(r.scale, 1.0);
    any_limiting = any_limiting || r.limiting;
  }
  EXPECT_TRUE(any_limiting);
}
