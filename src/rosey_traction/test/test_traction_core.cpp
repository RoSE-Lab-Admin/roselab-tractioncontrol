#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "rosey_traction/slip_table_lookup.hpp"
#include "rosey_traction/traction_core.hpp"

using rosey_traction::CoreConfig;
using rosey_traction::CycleInput;
using rosey_traction::CycleOutput;
using rosey_traction::TractionCore;
using rosey_traction::WheelInput;

namespace
{

CoreConfig makeConfig(bool enabled = false)
{
  CoreConfig c;
  for (const char * name : {"front_left_wheel_joint", "front_right_wheel_joint",
      "back_left_wheel_joint", "back_right_wheel_joint"})
  {
    c.wheels.push_back({name, 1.77, 0.15});
  }
  c.limiter.enabled = enabled;
  return c;
}

double wheelLoad(const CoreConfig & c) {return c.mass_kg * c.gravity / 4.0;}

// Motor current that produces the model torque at slip s for the default load.
double currentForSlip(const CoreConfig & c, double s)
{
  const auto t = rosey_traction::torqueAt(wheelLoad(c), s, c.model);
  EXPECT_TRUE(t.has_value());
  return *t / c.wheels[0].kt + c.wheels[0].i0;
}

CycleInput makeInput(double omega, double current)
{
  CycleInput in;
  in.wheels.assign(4, WheelInput{true, omega, current});
  in.joints_fresh = true;
  in.joints_new = true;
  in.imu_fresh = true;
  in.tilt_cos = 1.0;
  in.pitch = 0.0;
  in.have_command = true;
  in.cmd_angular_z = 0.0;
  in.dt = 0.1;
  return in;
}

bool hasWarning(const CycleOutput & out, const std::string & text)
{
  for (const auto & w : out.warnings) {
    if (w.find(text) != std::string::npos) {return true;}
  }
  return false;
}

}  // namespace

TEST(TractionCore, ConfigValidates)
{
  EXPECT_EQ(makeConfig().validate(), "");
  auto c = makeConfig();
  c.wheels[2].kt = 0.0;
  EXPECT_NE(c.validate(), "");
  c = makeConfig();
  c.wheels.clear();
  EXPECT_NE(c.validate(), "");
}

TEST(TractionCore, MockZeroCurrentGivesNoEstimate)
{
  // Mock hardware reports current = 0: no torque, so no estimate, and nothing breaks.
  TractionCore core(makeConfig());
  for (int i = 0; i < 3; ++i) {
    auto in = makeInput(1.0, 0.0);
    in.wheels[0].omega += 0.01 * i;  // keep it from looking frozen
    const auto out = core.step(in);
    ASSERT_EQ(out.wheels.size(), 4u);
    for (const auto & w : out.wheels) {
      EXPECT_FALSE(w.estimate_valid);
      EXPECT_TRUE(std::isnan(w.slip_est));
      EXPECT_FALSE(w.limiting);
      EXPECT_DOUBLE_EQ(w.torque, 0.0);
    }
    EXPECT_TRUE(std::isnan(out.error));
    EXPECT_DOUBLE_EQ(out.scale, 1.0);
    EXPECT_EQ(out.status(), "ok");
  }
}

TEST(TractionCore, NonFiniteTelemetryGivesNoEstimate)
{
  TractionCore core(makeConfig());
  auto in = makeInput(1.0, std::nan(""));
  in.wheels[1].omega = std::nan("");
  in.wheels[1].current = 2.0;
  const auto out = core.step(in);
  for (const auto & w : out.wheels) {EXPECT_FALSE(w.estimate_valid);}
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
}

TEST(TractionCore, RecoversSlipFromCurrent)
{
  const auto c = makeConfig();
  const double s_max = rosey_traction::sMax(wheelLoad(c));
  ASSERT_GT(s_max, 0.1);
  const double s_true = 0.5 * s_max;
  TractionCore core(c);
  const auto out = core.step(makeInput(1.0, currentForSlip(c, s_true)));
  for (const auto & w : out.wheels) {
    ASSERT_TRUE(w.estimate_valid) << w.joint_name;
    EXPECT_NEAR(w.slip_est, s_true, 0.005);
    EXPECT_NEAR(w.load, wheelLoad(c), 1e-9);
    EXPECT_NEAR(w.s_max, s_max, 1e-12);
    EXPECT_TRUE(std::isfinite(w.t_max));
    EXPECT_FALSE(w.limiting);
  }
  EXPECT_LT(out.error, 0.0);
  EXPECT_EQ(out.status(), "ok");
}

TEST(TractionCore, MonitorModeFlagsLimitingButKeepsScaleOne)
{
  const auto c = makeConfig(false);
  const double s_high = rosey_traction::sMax(wheelLoad(c)) + 0.1;
  TractionCore core(c);
  CycleOutput out;
  for (int i = 0; i < 20; ++i) {
    auto in = makeInput(1.0 + 0.001 * i, currentForSlip(c, s_high));
    out = core.step(in);
  }
  EXPECT_TRUE(out.wheels[0].estimate_valid);
  EXPECT_TRUE(out.wheels[0].limiting);
  EXPECT_GT(out.error, 0.0);
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
}

TEST(TractionCore, EnabledLimiterReducesScale)
{
  // The node keeps enabled = false until Phase 3; the core already supports it.
  const auto c = makeConfig(true);
  const double s_high = rosey_traction::sMax(wheelLoad(c)) + 0.1;
  TractionCore core(c);
  const auto out = core.step(makeInput(1.0, currentForSlip(c, s_high)));
  EXPECT_LT(out.scale, 1.0);
  EXPECT_GE(out.scale, c.limiter.k_min);
}

TEST(TractionCore, TurningBypassesAndResets)
{
  const auto c = makeConfig(true);
  const double s_high = rosey_traction::sMax(wheelLoad(c)) + 0.1;
  TractionCore core(c);
  core.step(makeInput(1.0, currentForSlip(c, s_high)));
  ASSERT_GT(core.limiter().integral(), 0.0);
  auto in = makeInput(1.001, currentForSlip(c, s_high));
  in.cmd_angular_z = -0.5;
  const auto out = core.step(in);
  EXPECT_TRUE(out.bypass_turning);
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
  EXPECT_DOUBLE_EQ(core.limiter().integral(), 0.0);
  EXPECT_TRUE(out.wheels[0].estimate_valid);  // still estimating while turning
}

TEST(TractionCore, StaleTelemetryGivesScaleOneAndWarning)
{
  const auto c = makeConfig(true);
  const double s_high = rosey_traction::sMax(wheelLoad(c)) + 0.1;
  TractionCore core(c);
  auto in = makeInput(1.0, currentForSlip(c, s_high));
  in.joints_fresh = false;
  const auto out = core.step(in);
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
  EXPECT_TRUE(hasWarning(out, "stale"));
  for (const auto & w : out.wheels) {EXPECT_FALSE(w.estimate_valid);}
}

TEST(TractionCore, MissingJointGivesWarning)
{
  TractionCore core(makeConfig());
  auto in = makeInput(1.0, 2.0);
  in.wheels[3].present = false;
  const auto out = core.step(in);
  EXPECT_TRUE(hasWarning(out, "back_right_wheel_joint"));
  for (const auto & w : out.wheels) {EXPECT_FALSE(w.estimate_valid);}
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
}

TEST(TractionCore, FrozenTelemetryWhileMoving)
{
  const auto c = makeConfig();
  TractionCore core(c);
  const auto in = makeInput(1.0, currentForSlip(c, 0.1));
  CycleOutput out;
  // First message sets the reference; each identical message after it counts once.
  for (int i = 0; i <= c.frozen_cycles - 1; ++i) {
    out = core.step(in);
    EXPECT_FALSE(hasWarning(out, "frozen")) << "message " << i;
  }
  out = core.step(in);
  EXPECT_TRUE(hasWarning(out, "frozen"));
  EXPECT_FALSE(out.wheels[0].estimate_valid);
  EXPECT_DOUBLE_EQ(out.scale, 1.0);

  // A changed value clears it.
  auto changed = in;
  changed.wheels[2].current += 1e-6;
  out = core.step(changed);
  EXPECT_FALSE(hasWarning(out, "frozen"));
  EXPECT_TRUE(out.wheels[0].estimate_valid);
}

TEST(TractionCore, FrozenIgnoresStoppedWheelsAndRepeatedTicks)
{
  const auto c = makeConfig();
  TractionCore core(c);
  // Stopped: identical zeros are normal.
  for (int i = 0; i < 3 * c.frozen_cycles; ++i) {
    EXPECT_FALSE(hasWarning(core.step(makeInput(0.0, 0.0)), "frozen"));
  }
  // Moving, but the timer ticked without a new message: does not count.
  auto in = makeInput(1.0, 2.0);
  core.step(in);
  in.joints_new = false;
  for (int i = 0; i < 3 * c.frozen_cycles; ++i) {
    EXPECT_FALSE(hasWarning(core.step(in), "frozen"));
  }
}

TEST(TractionCore, ReversingGivesNoEstimate)
{
  const auto c = makeConfig();
  TractionCore core(c);
  const auto out = core.step(makeInput(-1.0, currentForSlip(c, 0.1)));
  for (const auto & w : out.wheels) {
    EXPECT_FALSE(w.estimate_valid);
    EXPECT_LT(w.torque, 0.0);
  }
  EXPECT_TRUE(hasWarning(out, "reversing"));
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
}

TEST(TractionCore, ImuTiltScalesLoadAndStaleImuUsesFlat)
{
  const auto c = makeConfig();
  TractionCore core(c);
  auto in = makeInput(0.0, 0.0);
  in.tilt_cos = std::cos(0.2);
  in.pitch = 0.2;
  auto out = core.step(in);
  EXPECT_NEAR(out.wheels[0].load, wheelLoad(c) * std::cos(0.2), 1e-9);
  EXPECT_DOUBLE_EQ(out.pitch, 0.2);

  in.imu_fresh = false;
  out = core.step(in);
  EXPECT_NEAR(out.wheels[0].load, wheelLoad(c), 1e-9);
  EXPECT_DOUBLE_EQ(out.pitch, 0.0);
  EXPECT_TRUE(hasWarning(out, "imu stale"));
}

TEST(TractionCore, FrozenWhenCommandedButTelemetryStuck)
{
  // Telemetry stuck at zero while the rover is told to drive counts as frozen.
  const auto c = makeConfig();
  TractionCore core(c);
  auto in = makeInput(0.0, 0.0);
  in.cmd_linear_x = 0.2;
  CycleOutput out;
  for (int i = 0; i < c.frozen_cycles; ++i) {
    out = core.step(in);
    EXPECT_FALSE(hasWarning(out, "frozen")) << "message " << i;
  }
  out = core.step(in);
  EXPECT_TRUE(hasWarning(out, "frozen"));

  // Below moving_cmd_min it is just a stopped rover.
  TractionCore core2(c);
  in.cmd_linear_x = 0.5 * c.moving_cmd_min;
  for (int i = 0; i < 3 * c.frozen_cycles; ++i) {
    EXPECT_FALSE(hasWarning(core2.step(in), "frozen"));
  }
}

TEST(TractionCore, TorqueTermBacksUpInvalidEstimate)
{
  // Torque well above T(W, 0.9): the estimator gives up, the torque term still limits.
  const auto c = makeConfig(true);
  const auto t_09 = rosey_traction::torqueAt(wheelLoad(c), 0.9, c.model);
  ASSERT_TRUE(t_09.has_value());
  const double current = 1.3 * *t_09 / c.wheels[0].kt + c.wheels[0].i0;
  TractionCore core(c);
  const auto out = core.step(makeInput(1.0, current));
  for (const auto & w : out.wheels) {
    EXPECT_FALSE(w.estimate_valid);
    EXPECT_TRUE(w.limiting);
  }
  EXPECT_GT(out.error, 0.0);
  EXPECT_LT(out.scale, 1.0);
}

TEST(TractionCore, TorqueAndSlipTermsAgreeInSign)
{
  // The basis for "the torque term mainly backs up an invalid estimate": T rises with s, so
  // T > t_max exactly when s > s_max, for the loads the rover sees (flat to 30 deg tilt).
  const auto c = makeConfig();
  for (double tilt_cos : {1.0, std::cos(15.0 * rosey_traction::kDegToRad),
      std::cos(30.0 * rosey_traction::kDegToRad)})
  {
    const double w = wheelLoad(c) * tilt_cos;
    const double s_max = rosey_traction::sMax(w);
    const auto t_max = rosey_traction::torqueAt(w, s_max, c.model);
    ASSERT_TRUE(t_max.has_value());
    double t_prev = -1.0;
    for (double s = 0.02; s <= 0.9; s += 0.02) {
      const auto t = rosey_traction::torqueAt(w, s, c.model);
      ASSERT_TRUE(t.has_value());
      EXPECT_GT(*t, t_prev) << "W=" << w << " s=" << s;
      if (std::abs(s - s_max) > 1e-3) {
        EXPECT_EQ(*t > *t_max, s > s_max) << "W=" << w << " s=" << s;
      }
      t_prev = *t;
    }
  }
}
