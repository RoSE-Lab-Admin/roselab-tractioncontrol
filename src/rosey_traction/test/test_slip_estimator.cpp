#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>

#include "csv.hpp"
#include "rosey_traction/slip_estimator.hpp"

using rosey_traction::Params;
using rosey_traction::SlipEstimator;

namespace
{
constexpr double kRadToDeg = 180.0 / rosey_traction::kPi;
}

TEST(SlipEstimator, GoldenColdStart)
{
  const SlipEstimator est{Params()};
  const auto rows = test_csv::read(std::string(TEST_DATA_DIR) + "/golden_estimator.csv");
  ASSERT_EQ(rows.size(), 15u);
  for (const auto & row : rows) {
    SCOPED_TRACE("W=" + row.str("W_N") + " s_true=" + row.str("s_true"));
    const auto r = est.estimate(row.num("W_N"), row.num("T_Nm"));
    ASSERT_TRUE(r.valid);
    EXPECT_FALSE(r.used_fallback);
    EXPECT_NEAR(r.s, row.num("s_true"), 0.005);
    EXPECT_NEAR(r.theta_f * kRadToDeg, row.num("thf_true_rad") * kRadToDeg, 0.2);
  }
}

TEST(SlipEstimator, GoldenWarmStartFromNeighbour)
{
  // Warm start from a slightly wrong previous estimate, as the node will do each cycle.
  const SlipEstimator est{Params()};
  const auto rows = test_csv::read(std::string(TEST_DATA_DIR) + "/golden_estimator.csv");
  for (const auto & row : rows) {
    SCOPED_TRACE("W=" + row.str("W_N") + " s_true=" + row.str("s_true"));
    rosey_traction::WarmStart warm{row.num("s_true") + 0.05, row.num("thf_true_rad") * 1.05};
    const auto r = est.estimate(row.num("W_N"), row.num("T_Nm"), warm);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(r.s, row.num("s_true"), 0.005);
  }
}

namespace
{
// Seeded standard normal samples. Box-Muller on raw mt19937 output, because
// std::normal_distribution differs between standard libraries (macOS vs Linux CI).
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
}  // namespace

TEST(SlipEstimator, OnePercentNoise)
{
  // Noise model of the Octave demo: W and T each scaled by (1 + 0.01*randn).
  // For every golden case with s <= 0.3, the RMS slip error over many samples must be < 0.02.
  // Single samples can exceed 0.02 near s = 0.3, where dT/ds is small (see PROGRESS.md).
  const SlipEstimator est{Params()};
  const auto rows = test_csv::read(std::string(TEST_DATA_DIR) + "/golden_estimator.csv");
  Gaussian randn(12345u);
  const int n = 500;
  for (const auto & row : rows) {
    const double s_true = row.num("s_true");
    if (s_true > 0.3 + 1e-9) {continue;}
    SCOPED_TRACE("W=" + row.str("W_N") + " s_true=" + row.str("s_true"));
    double sum_sq = 0.0;
    int n_valid = 0;
    for (int i = 0; i < n; ++i) {
      const double w = row.num("W_N") * (1.0 + 0.01 * randn());
      const double t = row.num("T_Nm") * (1.0 + 0.01 * randn());
      const auto r = est.estimate(w, t);
      if (!r.valid) {continue;}
      ++n_valid;
      sum_sq += (r.s - s_true) * (r.s - s_true);
    }
    EXPECT_EQ(n_valid, n);
    const double rms = std::sqrt(sum_sq / std::max(n_valid, 1));
    EXPECT_LT(rms, 0.02);
    std::printf("  W=%s s_true=%.2f: rms slip error %.4f\n", row.str("W_N").c_str(), s_true, rms);
  }
}

TEST(SlipEstimator, InvalidInputDoesNotThrow)
{
  const SlipEstimator est{Params()};
  EXPECT_NO_THROW({
    EXPECT_FALSE(est.estimate(40.0, -0.5).valid);              // negative torque
    EXPECT_FALSE(est.estimate(1e6, 2.0).valid);                // load beyond model range
    EXPECT_FALSE(est.estimate(0.0, 2.0).valid);                // no load
    EXPECT_FALSE(est.estimate(std::nan(""), 2.0).valid);
    EXPECT_FALSE(est.estimate(40.0, std::nan("")).valid);
    EXPECT_FALSE(est.estimate(40.0, 1e3).valid);               // torque beyond T(s = 0.9)
  });
}

TEST(SlipEstimator, BisectionFallbackAgreesWithNewton)
{
  const SlipEstimator est{Params()};
  const auto rows = test_csv::read(std::string(TEST_DATA_DIR) + "/golden_estimator.csv");
  for (const auto & row : rows) {
    SCOPED_TRACE("W=" + row.str("W_N") + " s_true=" + row.str("s_true"));
    const auto r = est.bisection(row.num("W_N"), row.num("T_Nm"));
    ASSERT_TRUE(r.valid);
    EXPECT_TRUE(r.used_fallback);
    EXPECT_NEAR(r.s, row.num("s_true"), 0.005);
  }
}
