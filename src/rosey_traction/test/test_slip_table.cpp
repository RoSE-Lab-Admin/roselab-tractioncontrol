#include <gtest/gtest.h>

#include "rosey_traction/slip_table_lookup.hpp"

using namespace rosey_traction;

TEST(SlipTable, SMaxGridPointsExact)
{
  for (int i = 0; i < tableLoadCount(); ++i) {
    EXPECT_EQ(sMax(tableLoad(i)), tableSMax(i)) << "i=" << i;
  }
}

TEST(SlipTable, SMaxMidpointsAndClamp)
{
  for (int i = 0; i + 1 < tableLoadCount(); ++i) {
    const double mid = 0.5 * (tableLoad(i) + tableLoad(i + 1));
    EXPECT_NEAR(sMax(mid), 0.5 * (tableSMax(i) + tableSMax(i + 1)), 1e-12) << "i=" << i;
  }
  EXPECT_EQ(sMax(0.0), tableSMax(0));
  EXPECT_EQ(sMax(1e4), tableSMax(tableLoadCount() - 1));
}

TEST(SlipTable, SMinGridPointsExact)
{
  for (int i = 0; i < tableLoadCount(); ++i) {
    for (int j = 0; j < tableAlphaCount(); ++j) {
      const double expected = tableSMin(i, j) < 0.0 ? kInfeasible : tableSMin(i, j);
      EXPECT_EQ(sMin(tableLoad(i), tableAlpha(j)), expected) << "i=" << i << " j=" << j;
    }
  }
}

TEST(SlipTable, SMinBilinearMidpoint)
{
  // Cell W in [40, 45], alpha in [5, 10]: all four corners feasible.
  const double expected = 0.25 * (tableSMin(4, 1) + tableSMin(4, 2) + tableSMin(5, 1) + tableSMin(5, 2));
  EXPECT_NEAR(sMin(42.5, 7.5), expected, 1e-12);
  // Midpoint along one axis only.
  EXPECT_NEAR(sMin(40.0, 7.5), 0.5 * (tableSMin(4, 1) + tableSMin(4, 2)), 1e-12);
}

TEST(SlipTable, SMinInfeasibleCells)
{
  // W in [20, 25], alpha in [10, 15]: the alpha = 15 corners are -1.
  EXPECT_EQ(sMin(22.5, 12.5), kInfeasible);
  EXPECT_EQ(sMin(20.0, 15.0), kInfeasible);
  // W = 30, alpha 15 is feasible, but alpha 17.5 touches the -1 at alpha 20.
  EXPECT_GT(sMin(30.0, 15.0), 0.0);
  EXPECT_EQ(sMin(30.0, 17.5), kInfeasible);
  // Beyond the incline grid.
  EXPECT_EQ(sMin(50.0, 25.0), kInfeasible);
}

TEST(SlipTable, SMinClamps)
{
  EXPECT_EQ(sMin(40.0, -5.0), tableSMin(4, 0));   // downhill clamps to 0 deg
  EXPECT_EQ(sMin(10.0, 5.0), tableSMin(0, 1));    // light load clamps to 20 N row
  EXPECT_EQ(sMin(100.0, 5.0), tableSMin(tableLoadCount() - 1, 1));
}
