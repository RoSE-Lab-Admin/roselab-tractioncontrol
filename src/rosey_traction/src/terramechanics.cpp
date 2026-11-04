// Port of the functions in docs/traction/reference/rosey_terramechanics_octave.m.
// Comments name the Octave function each piece comes from.

#include "rosey_traction/terramechanics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rosey_traction
{

std::string Params::validate() const
{
  std::ostringstream err;
  if (!(r > 0.0)) {err << "r must be > 0. ";}
  if (!(b > 0.0)) {err << "b must be > 0. ";}
  if (!(h >= 0.0)) {err << "h must be >= 0. ";}
  if (!(K > 0.0)) {err << "K must be > 0. ";}
  if (n_int < 2 || n_int % 2 != 0) {err << "n_int must be even and >= 2. ";}
  if (!(thf_max > 1e-4 && thf_max < kPi / 2.0)) {err << "thf_max must be in (1e-4, pi/2). ";}
  return err.str();
}

// sinkExp
double sinkageExponent(double s, const Params & p)
{
  return p.n0 + p.n1 * s;
}

// normalStress, evaluated at one angle
double normalStress(double th, double thf, double s, const Params & p)
{
  const double N = sinkageExponent(s, p);
  const double thr = p.c3 * thf;
  const double thm = (p.c1 + p.c2 * s) * thf;
  const double k_eq = (p.kc / p.b + p.kphi) * 1e3 * std::pow(p.r, N);   // kPa -> Pa
  if (th >= thm) {
    // front region: theta_m <= theta <= theta_f
    return k_eq * std::pow(std::max(std::cos(th) - std::cos(thf), 0.0), N);
  }
  // rear region: theta_r <= theta < theta_m (thm > thr here, so no division by zero)
  const double th_eq = thf - (th - thr) / (thm - thr) * (thf - thm);
  return k_eq * std::pow(std::max(std::cos(th_eq) - std::cos(thf), 0.0), N);
}

// shearStress, evaluated at one angle (useThetaPrime = true)
double shearStress(double th, double thf, double s, double sig, const Params & p)
{
  const double z = p.r * (1.0 - std::cos(thf));
  const double th1 = std::acos(std::min((p.r - z) / p.Rj(), 1.0));
  const double j = p.rs() * ((th1 - th) - (1.0 - s) * (std::sin(th1) - std::sin(th)));
  return (p.c + sig * std::tan(p.phi)) * (1.0 - std::exp(-j / p.K));
}

// wheelForces. Simpson weights are applied while sampling, so no buffers are allocated.
Forces wheelForces(double thf, double s, const Params & p)
{
  const int n = p.n_int;
  const double thr = p.c3 * thf;
  const double dth = (thf - thr) / n;
  const double rs = p.rs();

  double sum_w = 0.0, sum_ft = 0.0, sum_r = 0.0, sum_tau = 0.0;
  for (int i = 0; i <= n; ++i) {
    const double th = (i == n) ? thf : thr + i * dth;   // linspace(thr, thf, n + 1)
    const double weight = (i == 0 || i == n) ? 1.0 : ((i % 2 == 1) ? 4.0 : 2.0);
    const double sig = normalStress(th, thf, s, p);
    const double tau = shearStress(th, thf, s, sig, p);
    const double c = std::cos(th);
    const double sn = std::sin(th);
    sum_w += weight * (p.r * sig * c + rs * tau * sn);
    sum_ft += weight * (rs * tau * c);
    sum_r += weight * (p.r * sig * sn);
    sum_tau += weight * tau;
  }
  const double k = dth / 3.0;

  Forces f;
  f.W = p.b * k * sum_w;
  f.FT = p.b * k * sum_ft;
  f.R = p.b * k * sum_r;
  f.DP = f.FT - f.R;
  f.T = rs * rs * p.b * k * sum_tau;
  return f;
}

// simpson
double simpson(const std::vector<double> & f, double x0, double x1)
{
  if (f.size() < 3 || (f.size() - 1) % 2 != 0) {
    return 0.0;
  }
  const size_t n = f.size() - 1;
  const double h = (x1 - x0) / static_cast<double>(n);
  double odd = 0.0, even = 0.0;
  for (size_t i = 1; i < n; ++i) {
    if (i % 2 == 1) {odd += f[i];} else {even += f[i];}
  }
  return h / 3.0 * (f.front() + 4.0 * odd + 2.0 * even + f.back());
}

// solveEntryAngle
std::optional<double> solveEntryAngle(double w_target, double s, const Params & p, double tol)
{
  if (!std::isfinite(w_target) || !std::isfinite(s)) {
    return std::nullopt;
  }
  double lo = 1e-4;
  double hi = p.thf_max;
  if (wheelForces(hi, s, p).W < w_target) {
    return std::nullopt;   // load beyond model range
  }
  for (int it = 0; it < 60; ++it) {
    const double mid = 0.5 * (lo + hi);
    if (wheelForces(mid, s, p).W > w_target) {hi = mid;} else {lo = mid;}
    if (hi - lo < tol) {break;}
  }
  return 0.5 * (lo + hi);
}

// torqueAt (rosey_current_tradestudy.m)
std::optional<double> torqueAt(double w, double s, const Params & p, double tol)
{
  const auto thf = solveEntryAngle(w, s, p, tol);
  if (!thf) {
    return std::nullopt;
  }
  return wheelForces(*thf, s, p).T;
}

}  // namespace rosey_traction
