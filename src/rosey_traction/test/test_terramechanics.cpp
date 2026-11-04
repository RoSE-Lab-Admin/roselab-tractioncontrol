// Compares the C++ port against golden values from docs/traction/reference/make_golden.m.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "csv.hpp"
#include "rosey_traction/terramechanics.hpp"

using rosey_traction::Params;

namespace
{

Params roseyParams() {return Params();}

// Ding Wh2 wheel as in make_golden.m (h = 15 mm), same soil.
Params dingParams()
{
  Params p;
  p.r = 0.15735;
  p.b = 0.165;
  p.h = 0.015;
  return p;
}

Params paramsFor(const std::string & wheel)
{
  return wheel == "rosey" ? roseyParams() : dingParams();
}

// 0.5% relative, or 1e-3 absolute when the golden value is small.
::testing::AssertionResult closeTo(double actual, double expected, const char * what)
{
  const double tol = (std::abs(expected) < 0.2) ? 1e-3 : 0.005 * std::abs(expected);
  if (std::abs(actual - expected) <= tol) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << what << ": got " << actual << ", expected " << expected
                                       << " (tol " << tol << ")";
}

}  // namespace

TEST(Terramechanics, DefaultParamsMatchOctave)
{
  const Params p;
  EXPECT_TRUE(p.validate().empty()) << p.validate();
  EXPECT_DOUBLE_EQ(p.rs(), 0.1515 + 0.5 * 0.013);
  EXPECT_DOUBLE_EQ(p.Rj(), 0.1515 + 0.013);
}

TEST(Terramechanics, ValidateRejectsOddIntervals)
{
  Params p;
  p.n_int = 61;
  EXPECT_FALSE(p.validate().empty());
}

TEST(Terramechanics, SimpsonIsExactForCubic)
{
  // Simpson is exact for polynomials up to degree 3.
  const int n = 10;
  std::vector<double> f(n + 1);
  for (int i = 0; i <= n; ++i) {
    const double x = 2.0 * i / n;
    f[i] = x * x * x - x + 1.0;
  }
  EXPECT_NEAR(rosey_traction::simpson(f, 0.0, 2.0), 4.0 - 2.0 + 2.0, 1e-12);
  EXPECT_EQ(rosey_traction::simpson(std::vector<double>{1.0, 2.0}, 0.0, 1.0), 0.0);
}

TEST(Terramechanics, GoldenForces)
{
  const auto rows = test_csv::read(std::string(TEST_DATA_DIR) + "/golden_forces.csv");
  ASSERT_EQ(rows.size(), 40u);
  for (const auto & row : rows) {
    const Params p = paramsFor(row.str("wheel"));
    const auto f = rosey_traction::wheelForces(row.num("thf_rad"), row.num("s"), p);
    SCOPED_TRACE(row.str("wheel") + " thf=" + row.str("thf_rad") + " s=" + row.str("s"));
    EXPECT_TRUE(closeTo(f.W, row.num("W_N"), "W"));
    EXPECT_TRUE(closeTo(f.DP, row.num("DP_N"), "DP"));
    EXPECT_TRUE(closeTo(f.T, row.num("T_Nm"), "T"));
    EXPECT_TRUE(closeTo(f.FT, row.num("FT_N"), "FT"));
    EXPECT_TRUE(closeTo(f.R, row.num("R_N"), "R"));
  }
}

TEST(Terramechanics, GoldenCurves)
{
  const auto rows = test_csv::read(std::string(TEST_DATA_DIR) + "/golden_curves.csv");
  ASSERT_EQ(rows.size(), 52u);
  for (const auto & row : rows) {
    const Params p = paramsFor(row.str("wheel"));
    const double s = row.num("s");
    SCOPED_TRACE(row.str("wheel") + " W=" + row.str("W_target_N") + " s=" + row.str("s"));
    const auto thf = rosey_traction::solveEntryAngle(row.num("W_target_N"), s, p);
    ASSERT_TRUE(thf.has_value());
    EXPECT_NEAR(*thf, row.num("thf_rad"), 1e-4);
    EXPECT_TRUE(closeTo(p.r * (1.0 - std::cos(*thf)), row.num("z_m"), "z"));
    const auto f = rosey_traction::wheelForces(*thf, s, p);
    EXPECT_TRUE(closeTo(f.DP, row.num("DP_N"), "DP"));
    EXPECT_TRUE(closeTo(f.T, row.num("T_Nm"), "T"));
  }
}

TEST(Terramechanics, EntryAngleOutOfRange)
{
  const Params p;
  EXPECT_FALSE(rosey_traction::solveEntryAngle(1e6, 0.1, p).has_value());
  EXPECT_FALSE(rosey_traction::solveEntryAngle(std::nan(""), 0.1, p).has_value());
  EXPECT_FALSE(rosey_traction::torqueAt(1e6, 0.1, p).has_value());
}
