#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegToRad = kPi / 180.0;
}  // namespace

class SPMJointStatePublisher : public rclcpp::Node
{
public:
  SPMJointStatePublisher()
  : Node("spm_joint_state_publisher"), previous_theta_{0.0, 0.0, 0.0}, ik_mode_("rrr")
  {
    // Default the stored quaternion to identity until the first message arrives.
    current_quat_.w = 1.0;
    current_quat_.x = 0.0;
    current_quat_.y = 0.0;
    current_quat_.z = 0.0;

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    orientation_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/spm/orientation_markers", 10);

    quaternion_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/spm/desired_orientation", 10,
      std::bind(&SPMJointStatePublisher::quaternion_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(50ms, std::bind(&SPMJointStatePublisher::publish_joint_states, this));

    this->declare_parameter<std::string>("ik_mode", "rrr");
    ik_mode_ = this->get_parameter("ik_mode").as_string();
    for (char & ch : ik_mode_) {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    const std::array<std::string, 9> valid_modes = {
      "continuity", "lll", "llr", "lrl", "lrr", "rll", "rlr", "rrl", "rrr"
    };
    const bool valid = std::find(valid_modes.begin(), valid_modes.end(), ik_mode_) != valid_modes.end();
    if (!valid) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid ik_mode '%s'. Falling back to continuity mode.",
        ik_mode_.c_str());
      ik_mode_ = "continuity";
    }

    // alpha_P: fixed DH twist angle of the proximal link (structural constant of the
    // manipulator in the paper's model: 60°).
    this->declare_parameter<double>("alpha_p_deg", 60.0);
    alpha_p_ = this->get_parameter("alpha_p_deg").as_double() * kDegToRad;

    // eta_i: fixed mounting angle of each leg about the base Z-axis. Defaults to the
    // 0/120/240 deg layout implied by the original v_home vectors in this node.
    this->declare_parameter<std::vector<double>>("eta_deg", std::vector<double>{0.0, 120.0, 240.0});
    const auto eta_deg = this->get_parameter("eta_deg").as_double_array();
    if (eta_deg.size() != 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "eta_deg parameter must have exactly 3 entries; using default 0/120/240.");
      eta_ = {0.0, 120.0 * kDegToRad, 240.0 * kDegToRad};
    } else {
      for (size_t i = 0; i < 3; ++i) {
        eta_[i] = eta_deg[i] * kDegToRad;
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing quaternion-based IK joint states on /joint_states "
      "(ik_mode=%s, alpha_P=%.2f deg, listening on /spm/desired_orientation)",
      ik_mode_.c_str(), alpha_p_ / kDegToRad);
  }

private:
  void quaternion_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    current_quat_ = *msg;
  }

  static double wrap_to_pi(double angle)
  {
    while (angle > kPi) {
      angle -= kTwoPi;
    }
    while (angle < -kPi) {
      angle += kTwoPi;
    }
    return angle;
  }

  static double unwrap_near(double angle, double reference)
  {
    return reference + wrap_to_pi(angle - reference);
  }

  static std::array<double, 3> select_continuous_solution(
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

  std::array<double, 3> select_mode_solution(
    const std::array<double, 3> & root_plus,
    const std::array<double, 3> & root_minus,
    const std::array<double, 3> & prev,
    const std::string & mode) const
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

  // Quaternion-derived rotation matrix (tool/platform frame -> reference frame),
  // matching the paper's Q[quat] convention with e0 = w, e1 = x, e2 = y, e3 = z.
  static std::array<std::array<double, 3>, 3> quaternion_to_rotation(
    double e0, double e1, double e2, double e3)
  {
    return {{
      {{e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3, 2.0 * (e1 * e2 - e0 * e3), 2.0 * (e1 * e3 + e0 * e2)}},
      {{2.0 * (e1 * e2 + e0 * e3), e0 * e0 - e1 * e1 + e2 * e2 - e3 * e3, 2.0 * (e2 * e3 - e0 * e1)}},
      {{2.0 * (e1 * e3 - e0 * e2), 2.0 * (e2 * e3 + e0 * e1), e0 * e0 - e1 * e1 - e2 * e2 + e3 * e3}}
    }};
  }

  static std::array<double, 3> mat_vec_mul(
    const std::array<std::array<double, 3>, 3> & m,
    const std::array<double, 3> & v)
  {
    return {
      m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
      m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
      m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]
    };
  }

  // Solves A*u^2 + B*u + C = 0 for both branches. Falls back to the linear case when
  // A ~ 0, and reports failure (false) when the orientation is unreachable for this leg
  // (negative discriminant, or a genuinely degenerate 0*u + 0 = 0 line).
  static bool solve_quadratic_u(double A, double B, double C, double eps, double & u_plus, double & u_minus)
  {
    if (std::abs(A) < eps) {
      if (std::abs(B) < eps) {
        return false;
      }
      const double u = -C / B;
      u_plus = u;
      u_minus = u;
      return true;
    }
    const double disc = B * B - 4.0 * A * C;
    if (disc < 0.0) {
      return false;
    }
    const double sq = std::sqrt(disc);
    u_plus = (-B + sq) / (2.0 * A);
    u_minus = (-B - sq) / (2.0 * A);
    return true;
  }

  // Inverse kinematics for the three motor angles theta_i, following the paper's
  // derivation: Weierstrass substitution u_i = tan(theta_i/2), with
  //   A_i * u_i^2 + B_i * u_i + C_i = 0
  // and A_i, B_i, C_i functions of the tool-orientation quaternion (e0,e1,e2,e3), the
  // leg mounting angle eta_i, and the proximal-link twist angle alpha_P.
  bool solve_inverse_kinematics(
    double e0, double e1, double e2, double e3,
    const std::array<double, 3> & prev_theta,
    std::array<double, 3> & theta_out) const
  {
    constexpr double eps = 1e-10;
    const double sinA = std::sin(alpha_p_);
    const double cosA = std::cos(alpha_p_);

    std::array<double, 3> root_plus{};
    std::array<double, 3> root_minus{};

    for (size_t i = 0; i < 3; ++i) {
      const double eta = eta_[i];
      const double c_eta = std::cos(eta);
      const double s_eta = std::sin(eta);
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

      // atan2(2u, 1-u^2) is equivalent to atan2(sin(theta), cos(theta)) up to the
      // common positive factor 1/(1+u^2), so it is safe to skip that division.
      root_plus[i] = std::atan2(2.0 * u_plus, 1.0 - u_plus * u_plus);
      root_minus[i] = std::atan2(2.0 * u_minus, 1.0 - u_minus * u_minus);
    }

    theta_out = select_mode_solution(root_plus, root_minus, prev_theta, ik_mode_);
    return true;
  }

  // Inverse kinematics for the passive distal-joint angles phi_i (Section 8.2).
  // Given the already-solved theta values, solves D_i*t^2 + E_i*t + F_i = 0
  // (Weierstrass t_i = tan(phi_i/2)) and selects the root in [0, pi] (sec. 8.4.2).
  bool solve_phi(
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

      // E coefficient — depends on alpha_P, eta and theta.
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

      // Select the root in [0, pi]; initial position is at pi/2 (90 deg).
      phi_out[i] = (phi1 >= 0.0 && phi1 <= kPi) ? phi1 : phi2;
    }

    return true;
  }

  void publish_joint_states()
  {
    const auto q = current_quat_;

    std::array<double, 3> theta{};
    if (!solve_inverse_kinematics(q.w, q.x, q.y, q.z, previous_theta_, theta)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "IK failed for the current orientation quaternion; keeping last joint state.");
      theta = previous_theta_;
    } else {
      previous_theta_ = theta;
    }

    std::array<double, 3> phi{};
    if (!solve_phi(q.w, q.x, q.y, q.z, theta, phi)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "phi IK failed for the current orientation; keeping last phi values.");
      phi = previous_phi_;
    } else {
      previous_phi_ = phi;
    }

    publish_orientation_markers(q.w, q.x, q.y, q.z);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = {
      "first_arm_joint",       "second_arm_joint",       "third_arm_joint",
      "first_arm_upper_joint", "second_arm_upper_joint", "third_arm_upper_joint"
    };
    msg.position = {theta[0], theta[1], theta[2], phi[0], phi[1], phi[2]};

    publisher_->publish(msg);
  }

  void publish_orientation_markers(double e0, double e1, double e2, double e3)
  {
    const auto r = quaternion_to_rotation(e0, e1, e2, e3);
    const std::array<std::array<double, 3>, 3> axes = {{
      {{1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}},
      {{0.0, 0.0, 1.0}}
    }};
    // Red = tool X axis, Green = tool Y axis, Blue = tool Z axis.
    constexpr std::array<std::array<float, 3>, 3> colors = {{
      {{1.0f, 0.2f, 0.2f}},
      {{0.2f, 1.0f, 0.2f}},
      {{0.2f, 0.4f, 1.0f}}
    }};
    constexpr double arrow_length = 0.0228;  // metres
    constexpr double z_offset = 0.0795;      // centre of rotation height above base_link

    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();

    for (size_t i = 0; i < 3; ++i) {
      const auto tip_dir = mat_vec_mul(r, axes[i]);

      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = "base_link";
      m.ns = "desired_orientation";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = z_offset;

      geometry_msgs::msg::Point tip;
      tip.x = tip_dir[0] * arrow_length;
      tip.y = tip_dir[1] * arrow_length;
      tip.z = tip_dir[2] * arrow_length + z_offset;

      m.points = {origin, tip};

      m.scale.x = 0.005;  // shaft diameter
      m.scale.y = 0.010;  // head diameter
      m.scale.z = 0.020;  // head length

      m.color.r = colors[i][0];
      m.color.g = colors[i][1];
      m.color.b = colors[i][2];
      m.color.a = 1.0f;

      array.markers.push_back(m);
    }

    orientation_marker_pub_->publish(array);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr orientation_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_sub_;

  geometry_msgs::msg::Quaternion current_quat_;
  std::array<double, 3> previous_theta_;
  std::array<double, 3> previous_phi_{kPi / 2.0, kPi / 2.0, kPi / 2.0};
  std::array<double, 3> eta_{};
  double alpha_p_{45.0 * kDegToRad};
  std::string ik_mode_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SPMJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
