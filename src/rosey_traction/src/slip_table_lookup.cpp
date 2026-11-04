#include "rosey_traction/slip_table_lookup.hpp"

#include <algorithm>
#include <cmath>

// Generated table, global namespace constexpr arrays. Included only in this file.
#include "rosey_traction/slip_table.h"

namespace rosey_traction
{

namespace
{

struct Bracket
{
  int i0;
  double t;   // weight of index i0 + 1, in [0, 1]
};

// Locate x on an ascending grid. x must already be inside [g[0], g[n-1]].
Bracket locate(const float * grid, int n, double x)
{
  int i = 0;
  while (i < n - 2 && x > static_cast<double>(grid[i + 1])) {
    ++i;
  }
  const double g0 = grid[i];
  const double g1 = grid[i + 1];
  double t = (x - g0) / (g1 - g0);
  t = std::clamp(t, 0.0, 1.0);
  return {i, t};
}

}  // namespace

double sMax(double w)
{
  if (!std::isfinite(w)) {
    return kInfeasible;
  }
  const double lo = LOAD_GRID_N[0];
  const double hi = LOAD_GRID_N[N_LOAD - 1];
  const Bracket b = locate(LOAD_GRID_N, N_LOAD, std::clamp(w, lo, hi));
  const double v0 = S_MAX[b.i0];
  const double v1 = S_MAX[b.i0 + 1];
  if (b.t == 0.0) {return v0;}
  if (b.t == 1.0) {return v1;}
  return (1.0 - b.t) * v0 + b.t * v1;
}

double sMin(double w, double alpha_deg)
{
  if (!std::isfinite(w) || !std::isfinite(alpha_deg)) {
    return kInfeasible;
  }
  if (alpha_deg > static_cast<double>(ALPHA_GRID_DEG[N_ALPHA - 1])) {
    return kInfeasible;
  }
  const double w_c = std::clamp(
    w, static_cast<double>(LOAD_GRID_N[0]), static_cast<double>(LOAD_GRID_N[N_LOAD - 1]));
  const double a_c = std::max(alpha_deg, static_cast<double>(ALPHA_GRID_DEG[0]));

  const Bracket bw = locate(LOAD_GRID_N, N_LOAD, w_c);
  const Bracket ba = locate(ALPHA_GRID_DEG, N_ALPHA, a_c);

  double sum = 0.0;
  for (int di = 0; di <= 1; ++di) {
    for (int dj = 0; dj <= 1; ++dj) {
      const double weight = (di ? bw.t : 1.0 - bw.t) * (dj ? ba.t : 1.0 - ba.t);
      if (weight == 0.0) {
        continue;
      }
      const double v = S_MIN[bw.i0 + di][ba.i0 + dj];
      if (v < 0.0) {
        return kInfeasible;
      }
      sum += weight * v;
    }
  }
  return sum;
}

int tableLoadCount() {return N_LOAD;}
int tableAlphaCount() {return N_ALPHA;}
double tableLoad(int i) {return LOAD_GRID_N[i];}
double tableAlpha(int j) {return ALPHA_GRID_DEG[j];}
double tableSMax(int i) {return S_MAX[i];}
double tableSMin(int i, int j) {return S_MIN[i][j];}

}  // namespace rosey_traction
