#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/arm_motor.hpp"
#include "redburi_msgs/msg/base_motor.hpp"

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode() : Node("serial_bridge_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    tx_rate_hz_ = declare_parameter<double>("tx_rate_hz", 50.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.2);

    base_sub_ = create_subscription<redburi_msgs::msg::BaseMotor>(
      "/base_motor",
      10,
      [this](const redburi_msgs::msg::BaseMotor::SharedPtr msg)
      {
        latest_base_ = *msg;
        has_base_ = true;
        last_base_time_ = now();
      });

    arm_sub_ = create_subscription<redburi_msgs::msg::ArmMotor>(
      "/arm_motor",
      10,
      [this](const redburi_msgs::msg::ArmMotor::SharedPtr msg)
      {
        latest_arm_ = *msg;
        has_arm_ = true;
        last_arm_time_ = now();
      });

    openSerial();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tx_rate_hz_));
    tx_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]()
      {
        sendFrame();
      });
  }

  ~SerialBridgeNode() override
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }
  }

private:
  std::string port_{};
  double tx_rate_hz_{};
  double command_timeout_sec_{};

  int serial_fd_{-1};

  bool has_base_{false};
  bool has_arm_{false};
  redburi_msgs::msg::BaseMotor latest_base_{};
  redburi_msgs::msg::ArmMotor latest_arm_{};
  rclcpp::Time last_base_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<redburi_msgs::msg::BaseMotor>::SharedPtr base_sub_;
  rclcpp::Subscription<redburi_msgs::msg::ArmMotor>::SharedPtr arm_sub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;

  static float safeFloat(float v)
  {
    return std::isfinite(v) ? v : 0.0f;
  }

  void openSerial()
  {
    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port_.c_str(), std::strerror(errno));
      return;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      return;
    }
  }

  void sendLine(const std::string & line)
  {
    if (serial_fd_ < 0) {
      return;
    }
    const ssize_t ret = ::write(serial_fd_, line.data(), line.size());
    if (ret < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "serial write failed: %s", std::strerror(errno));
    }
  }

  void sendFrame()
  {
    if (serial_fd_ < 0) {
      openSerial();
      if (serial_fd_ < 0) {
        return;
      }
    }

    redburi_msgs::msg::BaseMotor base{};
    redburi_msgs::msg::ArmMotor arm{};
    const auto now_time = now();
    const auto timeout = rclcpp::Duration::from_seconds(command_timeout_sec_);

    if (has_base_ && (now_time - last_base_time_) <= timeout) {
      base = latest_base_;
    }
    if (has_arm_ && (now_time - last_arm_time_) <= timeout) {
      arm = latest_arm_;
    }

    std::ostringstream b;
    b.setf(std::ios::fixed);
    b.precision(3);
    b << "B,"
      << safeFloat(base.motor_f_rpm) << ","
      << safeFloat(base.motor_r_rpm) << ","
      << safeFloat(base.motor_l_rpm) << ","
      << safeFloat(base.steer_deg) << "\n";

    std::ostringstream a;
    a.setf(std::ios::fixed);
    a.precision(3);
    a << "A,"
      << safeFloat(arm.joint_1_rpm) << ","
      << safeFloat(arm.joint_2_rpm) << ","
      << safeFloat(arm.joint_3_rpm) << ","
      << safeFloat(arm.joint_4_rpm) << ","
      << safeFloat(arm.joint_5_rpm) << ","
      << safeFloat(arm.joint_6_rpm) << ","
      << safeFloat(arm.gripper_rpm) << "\n";

    sendLine(b.str());
    sendLine(a.str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
