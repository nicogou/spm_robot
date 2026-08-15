#include "spm_robot/spm_ik.hpp"

#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

namespace spm_ik
{

SpmIK::SpmIK(double alpha_p, const std::array<double, 3> & eta)
: alpha_p_(alpha_p), eta_(eta)
{}

double SpmIK::wrap_to_pi(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double SpmIK::unwrap_near(double angle, double reference)
{
  return reference + wrap_to_pi(angle - reference);
}

std::array<double, 3> SpmIK::select_continuous_solution(
  const std::array<double, 3> & root_a,
  const std::array<double, 3> & root_b,
  const std::array<double, 3> & prev)
{
  std::array<double, 3> result{};
  for (size_t i = 0; i < 3; ++i) {
    const double a = unwrap_near(root_a[i], prev[i]);
    const double b = unwrap_near(root_b[i], prev[i]);
    result[i] = (std::abs(a - prev[i]) <= std::abs(b - prev[i])) ? a : b;
  }
  return result;
}

std::array<double, 3> SpmIK::select_mode_solution(
  const std::array<double, 3> & root_plus,
  const std::array<double, 3> & root_minus,
  const std::array<double, 3> & prev,
  const std::string & mode)
{
  if (mode == "continuity") {
    return select_continuous_solution(root_plus, root_minus, prev);
  }

  std::array<double, 3> result{};
  for (size_t i = 0; i < 3; ++i) {
    const double raw = (mode[i] == 'l') ? root_plus[i] : root_minus[i];
    result[i] = unwrap_near(raw, prev[i]);
  }
  return result;
}

bool SpmIK::solve_quadratic_u(
  double A, double B, double C, double eps,
  double & u_plus, double & u_minus)
{
  if (std::abs(A) < eps) {
    if (std::abs(B) < eps) {
      return false;
    }
    const double u = -C / B;
    u_plus  = u;
    u_minus = u;
    return true;
  }
  const double disc = B * B - 4.0 * A * C;
  if (disc < 0.0) {
    return false;
  }
  const double sq = std::sqrt(disc);
  u_plus  = (-B + sq) / (2.0 * A);
  u_minus = (-B - sq) / (2.0 * A);
  return true;
}

bool SpmIK::solve_theta(
  double e0, double e1, double e2, double e3,
  const std::array<double, 3> & prev_theta,
  const std::string & mode,
  std::array<double, 3> & theta_out) const
{
  constexpr double eps = 1e-10;
  const double sinA = std::sin(alpha_p_);
  const double cosA = std::cos(alpha_p_);

  std::array<double, 3> root_plus{};
  std::array<double, 3> root_minus{};

  for (size_t i = 0; i < 3; ++i) {
    const double eta    = eta_[i];
    const double c_eta  = std::cos(eta);
    const double s_eta  = std::sin(eta);
    const double c_2eta = std::cos(2.0 * eta);
    const double s_2eta = std::sin(2.0 * eta);
    const double cross_term = cosA * ((e0 * e1 + e2 * e3) * c_eta + (e0 * e2 - e1 * e3) * s_eta);

    const double A = 2.0 * (
      -e0 * e3 * sinA + e1 * e2 * c_2eta * sinA + cross_term
      - 0.5 * e1 * e1 * sinA * s_2eta + 0.5 * e2 * e2 * sinA * s_2eta);

    const double B = 2.0 * (
      -e0 * e0 * sinA + e3 * e3 * sinA + e1 * e1 * c_2eta * sinA - e2 * e2 * c_2eta * sinA
      + 2.0 * e1 * e2 * sinA * s_2eta);

    const double C = 2.0 * (
      e0 * e3 * sinA - e1 * e2 * c_2eta * sinA + cross_term
      + 0.5 * e1 * e1 * sinA * s_2eta - 0.5 * e2 * e2 * sinA * s_2eta);

    double u_plus = 0.0;
    double u_minus = 0.0;
    if (!solve_quadratic_u(A, B, C, eps, u_plus, u_minus)) {
      return false;
    }

    // atan2(2u, 1-u^2) recovers theta from the Weierstrass substitution u = tan(theta/2).
    root_plus[i]  = std::atan2(2.0 * u_plus,  1.0 - u_plus  * u_plus);
    root_minus[i] = std::atan2(2.0 * u_minus, 1.0 - u_minus * u_minus);
  }

  theta_out = select_mode_solution(root_plus, root_minus, prev_theta, mode);
  return true;
}

bool SpmIK::solve_phi(
  double e0, double e1, double e2, double e3,
  const std::array<double, 3> & theta,
  std::array<double, 3> & phi_out) const
{
  constexpr double eps = 1e-10;

  for (size_t i = 0; i < 3; ++i) {
    const double eta = eta_[i];
    const double th  = theta[i];

    const double c_th      = std::cos(th);
    const double s_th      = std::sin(th);
    const double c_2eta_th = std::cos(2.0 * eta + th);
    const double s_2eta_th = std::sin(2.0 * eta + th);

    // D and F coefficients (note F = -D).
    const double D =
      -(e0 * e0) * c_th      + (e3 * e3) * c_th
      + (e1 * e1) * c_2eta_th - (e2 * e2) * c_2eta_th
      - 2.0 * e0 * e3 * s_th + 2.0 * e1 * e2 * s_2eta_th;

    // E coefficient — depends on alpha_P, eta, and theta.
    const double E =
      -2.0 * (e0 * e2 - e1 * e3) * std::cos(alpha_p_ - eta)
      + 2.0 * (e0 * e2 - e1 * e3) * std::cos(alpha_p_ + eta)
      + 2.0 * e0 * e3 * std::cos(alpha_p_ - th)
      - 2.0 * e1 * e2 * std::cos(alpha_p_ - 2.0 * eta - th)
      + 2.0 * e0 * e3 * std::cos(alpha_p_ + th)
      - 2.0 * e1 * e2 * std::cos(alpha_p_ + 2.0 * eta + th)
      - 2.0 * (e0 * e1 + e2 * e3) * std::sin(alpha_p_ - eta)
      - 2.0 * (e0 * e1 + e2 * e3) * std::sin(alpha_p_ + eta)
      + (e0 * e0 - e3 * e3) * std::sin(alpha_p_ - th)
      - (e1 * e1 - e2 * e2) * std::sin(alpha_p_ - 2.0 * eta - th)
      - (e0 * e0 - e3 * e3) * std::sin(alpha_p_ + th)
      + (e1 * e1 - e2 * e2) * std::sin(alpha_p_ + 2.0 * eta + th);

    double t_plus = 0.0, t_minus = 0.0;
    if (!solve_quadratic_u(D, E, -D, eps, t_plus, t_minus)) {
      return false;
    }

    const double phi1 = std::atan2(2.0 * t_plus,  1.0 - t_plus  * t_plus);
    const double phi2 = std::atan2(2.0 * t_minus, 1.0 - t_minus * t_minus);

    // Select the root in [0, pi]; the home position is at pi/2.
    phi_out[i] = (phi1 >= 0.0 && phi1 <= kPi) ? phi1 : phi2;
  }

  return true;
}

}  // namespace spm_ik
