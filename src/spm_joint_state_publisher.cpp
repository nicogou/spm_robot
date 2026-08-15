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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "spm_robot/spm_ik.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
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

    // Only the passive upper-arm joints are published as joint states here —
    // the three actuated joints are owned by ros2_control's
    // joint_state_broadcaster once forward_position_controller is driving them.
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    // theta commands go to the position controller, in the same joint order
    // configured in spm_controllers.yaml (first/second/third_arm_joint).
    command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);
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
    const double alpha_p = this->get_parameter("alpha_p_deg").as_double() * kDegToRad;

    // eta_i: fixed mounting angle of each leg about the base Z-axis. Defaults to the
    // 0/120/240 deg layout implied by the original v_home vectors in this node.
    this->declare_parameter<std::vector<double>>("eta_deg", std::vector<double>{0.0, 120.0, 240.0});
    const auto eta_deg = this->get_parameter("eta_deg").as_double_array();
    std::array<double, 3> eta{};
    if (eta_deg.size() != 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "eta_deg parameter must have exactly 3 entries; using default 0/120/240.");
      eta = {0.0, 120.0 * kDegToRad, 240.0 * kDegToRad};
    } else {
      for (size_t i = 0; i < 3; ++i) {
        eta[i] = eta_deg[i] * kDegToRad;
      }
    }

    ik_solver_ = std::make_unique<spm_ik::SpmIK>(alpha_p, eta);

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing quaternion-based IK: theta -> /forward_position_controller/commands, "
      "phi -> /joint_states (ik_mode=%s, alpha_P=%.2f deg, listening on /spm/desired_orientation)",
      ik_mode_.c_str(), alpha_p / kDegToRad);
  }

private:
  void quaternion_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    current_quat_ = *msg;
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

  void publish_joint_states()
  {
    const auto q = current_quat_;

    std::array<double, 3> theta{};
    if (!ik_solver_->solve_theta(q.w, q.x, q.y, q.z, previous_theta_, ik_mode_, theta)) {
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
    if (!ik_solver_->solve_phi(q.w, q.x, q.y, q.z, theta, phi)) {
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

    // Actuated joints: send as a position command to forward_position_controller.
    // Order must match the `joints` list in spm_controllers.yaml.
    std_msgs::msg::Float64MultiArray command_msg;
    command_msg.data = {theta[0], theta[1], theta[2]};
    command_pub_->publish(command_msg);

    // Passive joints: publish directly, since ros2_control doesn't know about them.
    // robot_state_publisher merges this with joint_state_broadcaster's output on the
    // same /joint_states topic (it keeps a running map of joint name -> position).
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = {"first_arm_upper_joint", "second_arm_upper_joint", "third_arm_upper_joint"};
    msg.position = {phi[0], phi[1], phi[2]};

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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr orientation_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_sub_;

  geometry_msgs::msg::Quaternion current_quat_;
  std::array<double, 3> previous_theta_;
  std::array<double, 3> previous_phi_{kPi / 2.0, kPi / 2.0, kPi / 2.0};
  std::string ik_mode_;
  std::unique_ptr<spm_ik::SpmIK> ik_solver_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SPMJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
