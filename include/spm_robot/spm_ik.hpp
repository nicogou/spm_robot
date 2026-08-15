#ifndef SPM_ROBOT__SPM_IK_HPP_
#define SPM_ROBOT__SPM_IK_HPP_

#include <array>
#include <string>

namespace spm_ik
{

/// Spherical parallel manipulator inverse kinematics solver.
/// Configured once with the manipulator's structural constants, then queried
/// per-timestep for the actuated (theta) and passive (phi) joint angles.
class SpmIK
{
public:
  /// @param alpha_p  Proximal-link DH twist angle, radians.
  /// @param eta      Leg mounting angles about the base Z-axis, radians.
  SpmIK(double alpha_p, const std::array<double, 3> & eta);

  /// Solves for the three actuated joint angles theta given a platform orientation
  /// quaternion (e0=w, e1=x, e2=y, e3=z).  Returns false when the orientation is
  /// unreachable.  @p mode is "continuity" or a 3-char 'l'/'r' string per leg.
  bool solve_theta(
    double e0, double e1, double e2, double e3,
    const std::array<double, 3> & prev_theta,
    const std::string & mode,
    std::array<double, 3> & theta_out) const;

  /// Solves for the three passive distal-joint angles phi given the already-solved
  /// theta values.  Returns false when no valid solution exists.
  bool solve_phi(
    double e0, double e1, double e2, double e3,
    const std::array<double, 3> & theta,
    std::array<double, 3> & phi_out) const;

private:
  static double wrap_to_pi(double angle);
  static double unwrap_near(double angle, double reference);

  static std::array<double, 3> select_continuous_solution(
    const std::array<double, 3> & root_a,
    const std::array<double, 3> & root_b,
    const std::array<double, 3> & prev);

  static std::array<double, 3> select_mode_solution(
    const std::array<double, 3> & root_plus,
    const std::array<double, 3> & root_minus,
    const std::array<double, 3> & prev,
    const std::string & mode);

  // Solves A*u^2 + B*u + C = 0 via Weierstrass substitution u = tan(angle/2).
  static bool solve_quadratic_u(
    double A, double B, double C, double eps,
    double & u_plus, double & u_minus);

  double alpha_p_;
  std::array<double, 3> eta_;
};

}  // namespace spm_ik

#endif  // SPM_ROBOT__SPM_IK_HPP_
