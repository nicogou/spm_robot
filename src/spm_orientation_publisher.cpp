#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

struct Quat { double w, x, y, z; };

Quat qmul(const Quat & a, const Quat & b)
{
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}
}  // namespace

class SPMOrientationPublisher : public rclcpp::Node
{
public:
  SPMOrientationPublisher()
  : Node("spm_orientation_publisher"), angle_rad_(0.0)
  {
    this->declare_parameter<double>("angular_velocity_deg_s", 30.0);
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<double>("axis_x", 0.5);
    this->declare_parameter<double>("axis_y", 0.0);
    this->declare_parameter<double>("axis_z", 0.866);

    angular_velocity_rad_s_ =
      this->get_parameter("angular_velocity_deg_s").as_double() * kDegToRad;
    const double rate_hz = this->get_parameter("publish_rate_hz").as_double();

    const double ax = this->get_parameter("axis_x").as_double();
    const double ay = this->get_parameter("axis_y").as_double();
    const double az = this->get_parameter("axis_z").as_double();
    const double norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm < 1e-9) {
      RCLCPP_WARN(this->get_logger(), "Rotation axis is zero-length; defaulting to Z.");
      axis_ = {0.0, 0.0, 1.0};
    } else {
      axis_ = {ax / norm, ay / norm, az / norm};
    }

    // Precompute r_qT1: fixed tilt that maps the reference Z-axis onto the chosen axis.
    {
      const double nx = axis_[0], ny = axis_[1], nz = axis_[2];
      // Z × n_AoR = (0,0,1) × (nx,ny,nz) = (-ny, nx, 0)
      const double cx = -ny, cy = nx;
      const double cross_norm = std::sqrt(cx * cx + cy * cy);
      const double tilt_angle = std::acos(std::clamp(nz, -1.0, 1.0));
      if (cross_norm < 1e-9) {
        // n_AoR is (anti-)parallel to Z.
        r_qt1_ = (nz > 0.0) ? Quat{1.0, 0.0, 0.0, 0.0}   // identity
                             : Quat{0.0, 1.0, 0.0, 0.0};  // 180 deg around X
      } else {
        const double s = std::sin(tilt_angle / 2.0);
        r_qt1_ = {std::cos(tilt_angle / 2.0), cx / cross_norm * s, cy / cross_norm * s, 0.0};
      }
    }

    publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
      "/spm/desired_orientation", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = this->create_wall_timer(
      period, std::bind(&SPMOrientationPublisher::publish_quaternion, this));

    last_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "SAR mode: spinning at %.1f deg/s around axis (%.3f, %.3f, %.3f) on /spm/desired_orientation",
      angular_velocity_rad_s_ / kDegToRad, axis_[0], axis_[1], axis_[2]);
  }

private:
  void publish_quaternion()
  {
    const auto now = this->now();
    const double dt = (now - last_time_).seconds();
    last_time_ = now;

    angle_rad_ += angular_velocity_rad_s_ * dt;
    // Wrap to [-pi, pi] to prevent float drift over long runs.
    if (angle_rad_ >  kPi) {angle_rad_ -= 2.0 * kPi;}
    if (angle_rad_ < -kPi) {angle_rad_ += 2.0 * kPi;}

    const double s = std::sin(angle_rad_ / 2.0);
    const Quat r_qt2 = {std::cos(angle_rad_ / 2.0),
                        axis_[0] * s, axis_[1] * s, axis_[2] * s};
    const Quat q = qmul(r_qt2, r_qt1_);

    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w;  msg.x = q.x;  msg.y = q.y;  msg.z = q.z;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
  double angle_rad_;
  double angular_velocity_rad_s_;
  std::array<double, 3> axis_;
  Quat r_qt1_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SPMOrientationPublisher>());
  rclcpp::shutdown();
  return 0;
}
