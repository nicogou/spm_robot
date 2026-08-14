#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kDeg2Rad = kPi / 180.0;

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

// Rotate the body's Z-axis toward (tilt_x, tilt_y) from vertical.
// tilt_x > 0 → forward (+X), tilt_y > 0 → left (+Y), in ROS convention.
geometry_msgs::msg::Quaternion tilt_to_quat(double tx, double ty)
{
  geometry_msgs::msg::Quaternion q;
  const double theta = std::sqrt(tx * tx + ty * ty);
  if (theta < 1e-9) {
    q.w = 1.0; q.x = q.y = q.z = 0.0;
    return q;
  }
  // Rotation axis = Z × tilt_dir = (0,0,1) × (tx/θ, ty/θ, 0) = (-ty/θ, tx/θ, 0)
  const double s = std::sin(theta / 2.0);
  q.w = std::cos(theta / 2.0);
  q.x = (-ty / theta) * s;
  q.y = ( tx / theta) * s;
  q.z = 0.0;
  return q;
}

const char * kHelp =
  "\n--- SPM Teleop (NumLock ON) ---\n"
  "  7 (↖ fwd-left)   8 (↑ forward)   9 (↗ fwd-right)\n"
  "  4 (← left)       5 (■ center)    6 (→ right)\n"
  "  1 (↙ bwd-left)   2 (↓ backward)  3 (↘ bwd-right)\n"
  "  s  toggle SAR spin (around body Z / tilted axis)\n"
  "  z  toggle rotation around global Z\n"
  "  q  quit\n"
  "  Parameters: step_deg, max_tilt_deg, publish_rate_hz,\n"
  "              sar_angular_velocity_deg_s, z_angular_velocity_deg_s\n\n";
}  // namespace

class SPMTeleop : public rclcpp::Node
{
public:
  SPMTeleop()
  : Node("spm_teleop"), tilt_x_(0.0), tilt_y_(0.0)
  {
    this->declare_parameter<double>("step_deg", 2.0);
    this->declare_parameter<double>("max_tilt_deg", 45.0);
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<double>("sar_angular_velocity_deg_s", 30.0);
    this->declare_parameter<double>("z_angular_velocity_deg_s", 30.0);

    step_rad_     = this->get_parameter("step_deg").as_double()     * kDeg2Rad;
    max_tilt_rad_ = this->get_parameter("max_tilt_deg").as_double() * kDeg2Rad;
    const double rate_hz = this->get_parameter("publish_rate_hz").as_double();
    sar_angular_velocity_rad_s_ =
      this->get_parameter("sar_angular_velocity_deg_s").as_double() * kDeg2Rad;
    z_angular_velocity_rad_s_ =
      this->get_parameter("z_angular_velocity_deg_s").as_double() * kDeg2Rad;

    publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
      "/spm/desired_orientation", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = this->create_wall_timer(
      period, std::bind(&SPMTeleop::publish_quaternion, this));

    last_publish_time_ = this->now();

    std::printf("%s", kHelp);
    std::fflush(stdout);

    kb_thread_ = std::thread(&SPMTeleop::keyboard_loop, this);
  }

  ~SPMTeleop()
  {
    running_ = false;
    restore_terminal();
    if (kb_thread_.joinable()) {
      kb_thread_.join();
    }
  }

private:
  void setup_terminal()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_ERROR(this->get_logger(), "stdin is not a terminal — run this node directly from a shell.");
      return;
    }
    tcgetattr(STDIN_FILENO, &orig_termios_);
    struct termios raw = orig_termios_;
    // Only disable canonical mode and echo; keep OPOST so \n → \r\n and logs stay visible.
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;  // non-blocking read
    raw.c_cc[VTIME] = 2;  // 200 ms read timeout so the thread can check running_
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    is_tty_ = true;
  }

  void restore_terminal()
  {
    if (is_tty_) {
      tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios_);
    }
  }

  // Apply a unit-direction step (dx, dy) scaled to step_rad_, clamped to max_tilt_rad_.
  void apply_step(double dx, double dy)
  {
    const double len = std::sqrt(dx * dx + dy * dy);
    dx = dx / len * step_rad_;
    dy = dy / len * step_rad_;

    std::lock_guard<std::mutex> lock(tilt_mutex_);
    tilt_x_ += dx;
    tilt_y_ += dy;

    const double total = std::sqrt(tilt_x_ * tilt_x_ + tilt_y_ * tilt_y_);
    if (total > max_tilt_rad_) {
      tilt_x_ *= max_tilt_rad_ / total;
      tilt_y_ *= max_tilt_rad_ / total;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "tilt  x=%.1f°  y=%.1f°  |θ|=%.1f° / %.1f°",
      tilt_x_ * kRad2Deg,
      tilt_y_ * kRad2Deg,
      std::min(total, max_tilt_rad_) * kRad2Deg,
      max_tilt_rad_ * kRad2Deg);
  }

  void keyboard_loop()
  {
    setup_terminal();
    if (!is_tty_) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Keyboard thread ready.");
    while (running_) {
      char c = 0;
      if (read(STDIN_FILENO, &c, 1) <= 0) {
        continue;
      }

      switch (c) {
        // Cardinal directions
        case '8': apply_step(+1.0,  0.0); break;  // forward  (+X)
        case '2': apply_step(-1.0,  0.0); break;  // backward (-X)
        case '4': apply_step( 0.0, +1.0); break;  // left     (+Y)
        case '6': apply_step( 0.0, -1.0); break;  // right    (-Y)
        // Diagonals (normalised inside apply_step)
        case '7': apply_step(+1.0, +1.0); break;  // fwd-left
        case '9': apply_step(+1.0, -1.0); break;  // fwd-right
        case '1': apply_step(-1.0, +1.0); break;  // bwd-left
        case '3': apply_step(-1.0, -1.0); break;  // bwd-right
        // Center
        case '5': {
          std::lock_guard<std::mutex> lock(tilt_mutex_);
          tilt_x_ = tilt_y_ = 0.0;
          reset_spin_angles_.store(true);
          RCLCPP_INFO(this->get_logger(), "Centered (upright)");
          break;
        }
        case 's': case 'S': {
          const bool now_active = !sar_active_.load();
          sar_active_.store(now_active);
          RCLCPP_INFO(this->get_logger(), "SAR mode %s", now_active ? "ON" : "OFF");
          break;
        }
        case 'z': case 'Z': {
          const bool now_active = !z_active_.load();
          z_active_.store(now_active);
          RCLCPP_INFO(this->get_logger(), "Z-rotation mode %s", now_active ? "ON" : "OFF");
          break;
        }
        case 'q': case 'Q':
          RCLCPP_INFO(this->get_logger(), "Quit.");
          running_ = false;
          restore_terminal();
          rclcpp::shutdown();
          return;
        default: break;
      }
    }
    restore_terminal();
  }

  void publish_quaternion()
  {
    const rclcpp::Time now = this->now();
    const double dt = (now - last_publish_time_).seconds();
    last_publish_time_ = now;

    double tx, ty;
    {
      std::lock_guard<std::mutex> lock(tilt_mutex_);
      tx = tilt_x_;
      ty = tilt_y_;
    }

    const auto q_tilt_msg = tilt_to_quat(tx, ty);
    Quat q = {q_tilt_msg.w, q_tilt_msg.x, q_tilt_msg.y, q_tilt_msg.z};

    if (reset_spin_angles_.exchange(false)) {
      sar_angle_rad_ = 0.0;
      z_angle_rad_   = 0.0;
    }

    if (sar_active_.load()) {
      sar_angle_rad_ += sar_angular_velocity_rad_s_ * dt;
      if (sar_angle_rad_ >  kPi) { sar_angle_rad_ -= 2.0 * kPi; }
      if (sar_angle_rad_ < -kPi) { sar_angle_rad_ += 2.0 * kPi; }

      // AoR = body Z after tilt = where the tilted Z-axis points in world frame
      const double theta = std::sqrt(tx * tx + ty * ty);
      double ax, ay, az;
      if (theta < 1e-9) {
        ax = 0.0; ay = 0.0; az = 1.0;
      } else {
        ax = (tx / theta) * std::sin(theta);
        ay = (ty / theta) * std::sin(theta);
        az = std::cos(theta);
      }
      const double ss = std::sin(sar_angle_rad_ / 2.0);
      q = qmul({std::cos(sar_angle_rad_ / 2.0), ax * ss, ay * ss, az * ss}, q);
    }

    if (z_active_.load()) {
      z_angle_rad_ += z_angular_velocity_rad_s_ * dt;
      if (z_angle_rad_ >  kPi) { z_angle_rad_ -= 2.0 * kPi; }
      if (z_angle_rad_ < -kPi) { z_angle_rad_ += 2.0 * kPi; }

      const double sz = std::sin(z_angle_rad_ / 2.0);
      q = qmul({std::cos(z_angle_rad_ / 2.0), 0.0, 0.0, sz}, q);
    }

    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w; msg.x = q.x; msg.y = q.y; msg.z = q.z;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread kb_thread_;
  std::atomic<bool> running_{true};
  std::mutex tilt_mutex_;
  double tilt_x_;   // radians toward +X (forward)
  double tilt_y_;   // radians toward +Y (left)
  double step_rad_;
  double max_tilt_rad_;
  struct termios orig_termios_ {};
  bool is_tty_{false};
  std::atomic<bool> sar_active_{false};
  double sar_angle_rad_{0.0};
  double sar_angular_velocity_rad_s_{};
  std::atomic<bool> z_active_{false};
  double z_angle_rad_{0.0};
  double z_angular_velocity_rad_s_{};
  std::atomic<bool> reset_spin_angles_{false};
  rclcpp::Time last_publish_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SPMTeleop>());
  rclcpp::shutdown();
  return 0;
}
