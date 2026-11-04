#include <gtest/gtest.h>

#include <cmath>

#include "rosey_traction/current_to_torque.hpp"
#include "rosey_traction/traction_limiter.hpp"

using rosey_traction::LimiterConfig;
using rosey_traction::TractionLimiter;

namespace
{
LimiterConfig enabledConfig()
{
  LimiterConfig c;
  c.enabled = true;
  return c;
}
constexpr double kDt = 0.1;
}  // namespace

TEST(TractionLimiter, NoErrorKeepsFullSpeed)
{
  TractionLimiter lim(enabledConfig());
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(lim.update(0.0, kDt), 1.0);
    EXPECT_EQ(lim.update(-0.2, kDt), 1.0);
  }
}

TEST(TractionLimiter, SustainedErrorReducesScale)
{
  TractionLimiter lim(enabledConfig());
  double prev = 1.0;
  for (int i = 0; i < 20; ++i) {
    const double k = lim.update(0.05, kDt);
    EXPECT_LT(k, prev + 1e-12);
    prev = k;
  }
  EXPECT_LT(prev, 1.0 - 2.0 * 0.05);   // integral adds to the P term
}

TEST(TractionLimiter, NeverBelowKMin)
{
  TractionLimiter lim(enabledConfig());
  for (int i = 0; i < 1000; ++i) {
    EXPECT_GE(lim.update(1.0, kDt), lim.config().k_min);
  }
  EXPECT_DOUBLE_EQ(lim.scale(), lim.config().k_min);
  // Anti-windup: integral is capped where ki * I = 1 - k_min.
  EXPECT_LE(lim.integral(), (1.0 - lim.config().k_min) / lim.config().ki + 1e-12);
}

TEST(TractionLimiter, IntegratorBleedsOff)
{
  TractionLimiter lim(enabledConfig());
  for (int i = 0; i < 30; ++i) {lim.update(0.1, kDt);}
  const double i_peak = lim.integral();
  ASSERT_GT(i_peak, 0.0);
  lim.update(0.0, kDt);
  EXPECT_NEAR(lim.integral(), i_peak * std::exp(-lim.config().ki_bleed * kDt), 1e-12);
  for (int i = 0; i < 200; ++i) {lim.update(-0.1, kDt);}
  EXPECT_LT(lim.integral(), 1e-6);
  EXPECT_NEAR(lim.scale(), 1.0, 1e-6);
}

TEST(TractionLimiter, MonitorModeAlwaysOne)
{
  TractionLimiter lim;   // enabled = false by default
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(lim.update(0.5, kDt), 1.0);
  }
  EXPECT_EQ(lim.integral(), 0.0);
}

TEST(TractionLimiter, NonFiniteErrorAndBadDt)
{
  TractionLimiter lim(enabledConfig());
  lim.update(0.1, kDt);
  const double k = lim.scale();
  EXPECT_EQ(lim.update(0.1, 0.0), k);          // dt <= 0: no change
  EXPECT_GE(lim.update(std::nan(""), kDt), k);  // NaN: bleed, never cut harder
}

TEST(CurrentToTorque, Basics)
{
  using rosey_traction::currentToTorque;
  EXPECT_DOUBLE_EQ(currentToTorque(1.15, 2.0, 1.77, 0.15), 1.77);
  EXPECT_DOUBLE_EQ(currentToTorque(-1.15, 2.0, 1.77, 0.15), 1.77);   // sign from omega
  EXPECT_DOUBLE_EQ(currentToTorque(1.15, -2.0, 1.77, 0.15), -1.77);
  EXPECT_EQ(currentToTorque(0.10, 2.0, 1.77, 0.15), 0.0);            // below I0
  EXPECT_EQ(currentToTorque(1.0, 0.0, 1.77, 0.15), 0.0);
  EXPECT_EQ(currentToTorque(std::nan(""), 2.0, 1.77, 0.15), 0.0);
}
