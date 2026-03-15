#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/arm_motor.hpp"
#include "redburi_msgs/msg/base_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode() : Node("serial_bridge_node")
  {
    candidate_ports_ = declare_parameter<std::vector<std::string>>(
      "candidate_ports", std::vector<std::string>{"/dev/ttyUSB0"});
    tx_rate_hz_ = declare_parameter<double>("tx_rate_hz", 50.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.2);

    base_sub_ = create_subscription<redburi_msgs::msg::BaseCommand>(
      "/base_cmd",
      10,
      [this](const redburi_msgs::msg::BaseCommand::SharedPtr msg)
      {
        latest_base_ = *msg;
        has_base_ = true;
        last_base_time_ = now();
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "base_cmd rx motor_rpm=%.3f steer_deg=%.3f",
          static_cast<double>(latest_base_.motor_rpm),
          static_cast<double>(latest_base_.target_steer_deg));
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

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    steer_state_pub_ = create_publisher<std_msgs::msg::Float32>("/steer_state", 10);
    raw_rx_line_pub_ = create_publisher<std_msgs::msg::String>("/stm32_rx_line", 10);

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
  std::vector<std::string> candidate_ports_{};
  std::string active_port_{};
  double tx_rate_hz_{};
  double command_timeout_sec_{};

  int serial_fd_{-1};

  bool has_base_{false};
  bool has_arm_{false};
  redburi_msgs::msg::BaseCommand latest_base_{};
  redburi_msgs::msg::ArmMotor latest_arm_{};
  rclcpp::Time last_base_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<redburi_msgs::msg::BaseCommand>::SharedPtr base_sub_;
  rclcpp::Subscription<redburi_msgs::msg::ArmMotor>::SharedPtr arm_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_rx_line_pub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
  std::string rx_line_buffer_{};
  static constexpr size_t kArmJointCount = 6;
  const std::vector<std::string> joint_names_{
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6"};

  static float safeFloat(float v)
  {
    return std::isfinite(v) ? v : 0.0f;
  }

  static int safeInt(float v)
  {
    return static_cast<int>(std::lround(safeFloat(v)));
  }

  static bool parseCsvFloats(
    const std::string & line, char head, size_t expected_count, std::vector<float> & out_values)
  {
    if (line.size() < 3 || line[0] != head || line[1] != ',') {
      return false;
    }

    out_values.clear();
    out_values.reserve(expected_count);

    const char * cursor = line.c_str() + 2;
    for (size_t i = 0; i < expected_count; ++i) {
      char * end_ptr = nullptr;
      const float value = std::strtof(cursor, &end_ptr);
      if (end_ptr == cursor) {
        return false;
      }
      out_values.push_back(safeFloat(value));

      if (i + 1 < expected_count) {
        if (*end_ptr != ',') {
          return false;
        }
        cursor = end_ptr + 1;
      } else {
        if (*end_ptr != '\0') {
          return false;
        }
      }
    }

    return true;
  }

  bool configureSerialFd(int fd, const std::string & port_name)
  {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr(%s) failed: %s", port_name.c_str(), std::strerror(errno));
      return false;
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

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr(%s) failed: %s", port_name.c_str(), std::strerror(errno));
      return false;
    }

    int modem_bits = 0;
    if (::ioctl(fd, TIOCMGET, &modem_bits) == 0) {
      modem_bits |= (TIOCM_DTR | TIOCM_RTS);
      if (::ioctl(fd, TIOCMSET, &modem_bits) != 0) {
        RCLCPP_WARN(
          get_logger(), "ioctl(TIOCMSET, %s) failed: %s", port_name.c_str(), std::strerror(errno));
      }
    } else {
      RCLCPP_WARN(
        get_logger(), "ioctl(TIOCMGET, %s) failed: %s", port_name.c_str(), std::strerror(errno));
    }

    return true;
  }

  void openSerial()
  {
    std::ostringstream error_stream;

    for (const auto & candidate_port : candidate_ports_) {
      if (candidate_port.empty()) {
        continue;
      }

      const int fd = ::open(candidate_port.c_str(), O_RDWR | O_NOCTTY);
      if (fd < 0) {
        error_stream << "open(" << candidate_port << ") failed: " << std::strerror(errno) << "; ";
        continue;
      }

      if (!configureSerialFd(fd, candidate_port)) {
        ::close(fd);
        continue;
      }

      serial_fd_ = fd;
      active_port_ = candidate_port;
      RCLCPP_INFO(get_logger(), "connected serial bridge on %s", active_port_.c_str());
      return;
    }

    serial_fd_ = -1;
    active_port_.clear();
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "failed to open any serial port candidate: %s",
      error_stream.str().c_str());
  }

  void sendLine(const std::string & line)
  {
    if (serial_fd_ < 0) {
      return;
    }

    size_t total = 0;
    while (total < line.size()) {
      const ssize_t ret = ::write(serial_fd_, line.data() + total, line.size() - total);
      if (ret < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "serial write failed: %s", std::strerror(errno));
        ::close(serial_fd_);
        serial_fd_ = -1;
        active_port_.clear();
        return;
      }
      if (ret == 0) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "serial write returned 0 bytes");
        ::close(serial_fd_);
        serial_fd_ = -1;
        active_port_.clear();
        return;
      }
      total += static_cast<size_t>(ret);
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

    redburi_msgs::msg::BaseCommand base{};
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
    b << "B,"
      << safeInt(base.motor_rpm) << ","
      << safeInt(base.target_steer_deg) << "\n";

    std::ostringstream a;
    a << "A,"
      << safeInt(arm.joint_1_rpm) << ","
      << safeInt(arm.joint_2_rpm) << ","
      << safeInt(arm.joint_3_rpm) << ","
      << safeInt(arm.joint_4_rpm) << ","
      << safeInt(arm.joint_5_rpm) << ","
      << safeInt(arm.joint_6_rpm) << ","
      << safeInt(arm.gripper_rpm) << "\n";

    sendLine(b.str());
    sendLine(a.str());
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "tx %s",
      b.str().c_str());
    pollRxLines();
  }

  void pollRxLines()
  {
    if (serial_fd_ < 0) {
      return;
    }

    char buf[256]{};
    while (true) {
      const ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          const char c = buf[i];
          if (c == '\r') {
            continue;
          }
          if (c == '\n') {
            if (!rx_line_buffer_.empty()) {
              processReceivedLine(rx_line_buffer_);
              rx_line_buffer_.clear();
            }
            continue;
          }

          if (rx_line_buffer_.size() < 255) {
            rx_line_buffer_.push_back(c);
          } else {
            // Drop too-long line and wait for next newline.
            rx_line_buffer_.clear();
          }
        }
        continue;
      }

      if (n == 0) {
        break;
      }

      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "serial read failed: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      active_port_.clear();
      rx_line_buffer_.clear();
      break;
    }
  }

  void processReceivedLine(const std::string & line)
  {
    if (line.empty()) {
      return;
    }

    std_msgs::msg::String raw{};
    raw.data = line;
    raw_rx_line_pub_->publish(raw);

    std::vector<float> values{};

    if (line[0] == 'J') {
      if (!parseCsvFloats(line, 'J', kArmJointCount, values) &&
        !parseCsvFloats(line, 'J', kArmJointCount + 1, values))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "failed to parse J line: %s", line.c_str());
        return;
      }
      if (values.size() > kArmJointCount) {
        values.resize(kArmJointCount);
      }
      publishJointStates(values);
      return;
    }

    if (line[0] == 'S') {
      if (!parseCsvFloats(line, 'S', 1, values)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "failed to parse S line: %s", line.c_str());
        return;
      }
      publishSteerState(values[0]);
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "unknown rx line: %s", line.c_str());
  }

  void publishJointStates(const std::vector<float> & values)
  {
    sensor_msgs::msg::JointState msg{};
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position.reserve(values.size());
    for (float value : values) {
      msg.position.push_back(static_cast<double>(value));
    }
    joint_state_pub_->publish(msg);
  }

  void publishSteerState(float steer_deg)
  {
    std_msgs::msg::Float32 msg{};
    msg.data = steer_deg;
    steer_state_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
