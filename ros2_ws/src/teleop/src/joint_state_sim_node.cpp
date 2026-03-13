#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/arm_motor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateSimNode : public rclcpp::Node
{
public:
  JointStateSimNode() : Node("joint_state_sim_node")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 60.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.2);

    arm_sub_ = create_subscription<redburi_msgs::msg::ArmMotor>(
      "/arm_motor",
      10,
      [this](const redburi_msgs::msg::ArmMotor::SharedPtr msg)
      {
        latest_rpm_[0] = safeRpm(msg->joint_1_rpm);
        latest_rpm_[1] = safeRpm(msg->joint_2_rpm);
        latest_rpm_[2] = safeRpm(msg->joint_3_rpm);
        latest_rpm_[3] = safeRpm(msg->joint_4_rpm);
        latest_rpm_[4] = safeRpm(msg->joint_5_rpm);
        latest_rpm_[5] = safeRpm(msg->joint_6_rpm);
        has_arm_command_ = true;
        last_command_time_ = std::chrono::steady_clock::now();
      });

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    last_update_time_ = std::chrono::steady_clock::now();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]()
      {
        updateAndPublish();
      });
  }

private:
  static constexpr size_t kJointCount = 6;
  static constexpr double kTwoPi = 6.28318530717958647692;

  static double safeRpm(float rpm)
  {
    return std::isfinite(rpm) ? static_cast<double>(rpm) : 0.0;
  }

  static double rpmToRadPerSec(double rpm)
  {
    return rpm * kTwoPi / 60.0;
  }

  void updateAndPublish()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt =
      std::chrono::duration<double>(current_time - last_update_time_).count();
    last_update_time_ = current_time;

    std::array<double, kJointCount> commanded_rpm{};
    if (
      has_arm_command_ &&
      std::chrono::duration<double>(current_time - last_command_time_).count() <=
      command_timeout_sec_)
    {
      commanded_rpm = latest_rpm_;
    }

    sensor_msgs::msg::JointState msg{};
    msg.header.stamp = now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.position.reserve(kJointCount);
    msg.velocity.reserve(kJointCount);

    for (size_t i = 0; i < kJointCount; ++i) {
      const double velocity = rpmToRadPerSec(commanded_rpm[i]);
      positions_[i] += velocity * dt;
      msg.position.push_back(positions_[i]);
      msg.velocity.push_back(velocity);
    }

    joint_pub_->publish(msg);
  }

  double publish_rate_hz_{};
  double command_timeout_sec_{};
  bool has_arm_command_{false};
  std::array<double, kJointCount> latest_rpm_{};
  std::array<double, kJointCount> positions_{};
  std::array<std::string, kJointCount> joint_names_{
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6"};
  std::chrono::steady_clock::time_point last_update_time_{};
  std::chrono::steady_clock::time_point last_command_time_{};

  rclcpp::Subscription<redburi_msgs::msg::ArmMotor>::SharedPtr arm_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSimNode>());
  rclcpp::shutdown();
  return 0;
}
