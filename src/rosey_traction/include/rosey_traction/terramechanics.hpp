// Terramechanics wheel model for RoSEy traction control.
//
// Port of docs/traction/reference/rosey_terramechanics_octave.m (functions section).
// Model: Ding et al. 2009 (IROS), Eqs. 16 to 22.
//   Normal stress: Bekker pressure-sinkage with Wong-Reece stress distribution.
//   Shear stress:  Janosi-Hanamoto, shearing radius r_s = r + lambda_s*h.
//   Grousers:      lug-entry angle theta_f' (Ding Eq. 17) in the shear displacement.
//
// The Octave script has switches useSlipSinkage and useThetaPrime. This port implements the
// validated variant only (both true), which is also what make_golden.m uses.
//
// Units: SI (m, N, Pa, rad). kc and kphi keep the Octave units (kPa based); the 1e3 factor
// is applied inside normalStress exactly as in the script.
//
// No ROS includes: this file must stay plain C++17.

#ifndef ROSEY_TRACTION__TERRAMECHANICS_HPP_
#define ROSEY_TRACTION__TERRAMECHANICS_HPP_

#include <optional>
#include <string>
#include <vector>

namespace rosey_traction
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

struct Params
{
  // Wheel
  double r = 0.1515;          // wheel radius [m]
  double b = 0.05158;         // wheel width [m]
  double h = 0.013;           // grouser height [m]
  double lambda_s = 0.5;      // lug shearing coefficient [-]

  // Soil (Ding et al. 2011 Table 2, n0/n1/K fitted from Ding 2009 Wh2)
  double kc = 15.6;           // cohesive modulus of sinkage [kPa/m^(n-1)]
  double kphi = 2407.4;       // frictional modulus of sinkage [kPa/m^n]
  double n0 = 0.81;           // static sinkage exponent [-]
  double n1 = 1.16;           // slip-sinkage coefficient [-]
  double c = 250.0;           // cohesion [Pa]
  double phi = 31.9 * kDegToRad;  // internal friction angle [rad]
  double K = 0.011;           // shear deformation modulus [m]
  double c1 = 0.5;            // max stress angle coefficient [-]
  double c2 = -0.3;           // max stress angle slip coefficient [-]
  double c3 = 0.0;            // exit angle constant, theta_r = c3*theta_f [-]

  // Numerics
  int n_int = 60;                     // Simpson intervals, must be even
  double thf_max = 80.0 * kDegToRad;  // upper bound for entry angle [rad]

  double rs() const {return r + lambda_s * h;}   // shearing radius [m]
  double Rj() const {return r + h;}              // lug tip radius used for theta_f' [m]

  // Empty string if the parameters are usable, otherwise a description of the problem.
  std::string validate() const;
};

struct Forces
{
  double W = 0.0;    // vertical load [N]
  double DP = 0.0;   // drawbar pull [N]
  double T = 0.0;    // driving torque [N m]
  double FT = 0.0;   // gross tractive force [N]
  double R = 0.0;    // compaction resistance [N]
};

// Sinkage exponent N = n0 + n1*s (Ding 2009 Eq. 16).
double sinkageExponent(double s, const Params & p);

// Normal stress sigma(theta) [Pa] (Ding 2009 Eq. 20).
double normalStress(double th, double thf, double s, const Params & p);

// Shear stress tau(theta) [Pa] for a given normal stress sig (Ding 2009 Eq. 21, with theta_f').
double shearStress(double th, double thf, double s, double sig, const Params & p);

// W, DP, T, FT, R for entry angle thf and slip s (Ding 2009 Eq. 22).
// Integrates over theta in [c3*thf, thf] with composite Simpson, p.n_int intervals.
Forces wheelForces(double thf, double s, const Params & p);

// Composite Simpson's rule on evenly spaced samples f over [x0, x1].
// f.size() - 1 must be even and at least 2; returns 0 otherwise.
double simpson(const std::vector<double> & f, double x0, double x1);

// Bisection for theta_f such that W(theta_f, s) = w_target, on [1e-4, thf_max].
// Returns std::nullopt when the load is beyond the model range (W(thf_max) < w_target)
// or the target is not finite.
// tol is the bracket width at which bisection stops (make_golden.m uses 1e-10).
std::optional<double> solveEntryAngle(
  double w_target, double s, const Params & p, double tol = 1e-10);

// Torque at load w and slip s: solveEntryAngle, then wheelForces. nullopt if out of range.
std::optional<double> torqueAt(double w, double s, const Params & p, double tol = 1e-10);

}  // namespace rosey_traction

#endif  // ROSEY_TRACTION__TERRAMECHANICS_HPP_
