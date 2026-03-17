#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/arm_motor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmJointStateEstimatorNode : public rclcpp::Node
{
public:
  ArmJointStateEstimatorNode() : Node("arm_joint_state_estimator_node")
  {
    arm_command_topic_ = declare_parameter<std::string>("arm_command_topic", "/arm_motor");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 60.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.2);

    const std::vector<double> initial_positions = declare_parameter<std::vector<double>>(
      "initial_positions",
      {0.0, -0.2, -0.2, 0.0, -0.2, 0.0});
    const std::vector<double> joint_lower_limits = declare_parameter<std::vector<double>>(
      "joint_lower_limits",
      {-1.3963, -2.7925, -2.7925, -1.3963, -2.7925, -1.3963});
    const std::vector<double> joint_upper_limits = declare_parameter<std::vector<double>>(
      "joint_upper_limits",
      {1.3963, 0.0, 0.0, 1.3963, 0.0, 1.3963});

    estimated_positions_ = vectorToArray(
      initial_positions,
      {0.0, -0.2, -0.2, 0.0, -0.2, 0.0},
      "initial_positions");
    joint_lower_limits_ = vectorToArray(
      joint_lower_limits,
      {-1.3963, -2.7925, -2.7925, -1.3963, -2.7925, -1.3963},
      "joint_lower_limits");
    joint_upper_limits_ = vectorToArray(
      joint_upper_limits,
      {1.3963, 0.0, 0.0, 1.3963, 0.0, 1.3963},
      "joint_upper_limits");

    last_update_time_ = now();
    last_command_time_ = last_update_time_;

    arm_sub_ = create_subscription<redburi_msgs::msg::ArmMotor>(
      arm_command_topic_,
      10,
      [this](const redburi_msgs::msg::ArmMotor::SharedPtr msg)
      {
        armCommandCallback(msg);
      });

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]()
      {
        publishJointState();
      });
  }

private:
  static constexpr size_t kJointCount = 6;
  static constexpr double kRpmToRadPerSec = 2.0 * M_PI / 60.0;

  std::array<double, kJointCount> commanded_rpm_{};
  std::array<double, kJointCount> estimated_positions_{};
  std::array<double, kJointCount> joint_lower_limits_{};
  std::array<double, kJointCount> joint_upper_limits_{};
  const std::array<std::string, kJointCount> joint_names_{
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6"};

  std::string arm_command_topic_{};
  std::string joint_state_topic_{};
  std::string frame_id_{};
  double publish_rate_hz_{};
  double command_timeout_sec_{};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<redburi_msgs::msg::ArmMotor>::SharedPtr arm_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  static std::array<double, kJointCount> vectorToArray(
    const std::vector<double> & values,
    const std::array<double, kJointCount> & fallback,
    const std::string & parameter_name)
  {
    std::array<double, kJointCount> result = fallback;

    if (values.size() != kJointCount) {
      RCLCPP_WARN(
        rclcpp::get_logger("arm_joint_state_estimator_node"),
        "parameter '%s' expected %zu values, got %zu; using defaults",
        parameter_name.c_str(),
        kJointCount,
        values.size());
      return result;
    }

    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  void armCommandCallback(const redburi_msgs::msg::ArmMotor::SharedPtr msg)
  {
    commanded_rpm_[0] = static_cast<double>(msg->joint_1_rpm);
    commanded_rpm_[1] = static_cast<double>(msg->joint_2_rpm);
    commanded_rpm_[2] = static_cast<double>(msg->joint_3_rpm);
    commanded_rpm_[3] = static_cast<double>(msg->joint_4_rpm);
    commanded_rpm_[4] = static_cast<double>(msg->joint_5_rpm);
    commanded_rpm_[5] = static_cast<double>(msg->joint_6_rpm);
    last_command_time_ = now();
  }

  void publishJointState()
  {
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0) {
      dt = 0.0;
    }

    std::array<double, kJointCount> joint_velocity_rad_per_sec{};
    const bool command_is_fresh =
      (current_time - last_command_time_) <= rclcpp::Duration::from_seconds(command_timeout_sec_);

    for (size_t i = 0; i < kJointCount; ++i) {
      const double velocity =
        command_is_fresh ? commanded_rpm_[i] * kRpmToRadPerSec : 0.0;
      joint_velocity_rad_per_sec[i] = velocity;
      estimated_positions_[i] = std::clamp(
        estimated_positions_[i] + velocity * dt,
        joint_lower_limits_[i],
        joint_upper_limits_[i]);
    }

    sensor_msgs::msg::JointState msg{};
    msg.header.stamp = current_time;
    msg.header.frame_id = frame_id_;
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.position.assign(estimated_positions_.begin(), estimated_positions_.end());
    msg.velocity.assign(joint_velocity_rad_per_sec.begin(), joint_velocity_rad_per_sec.end());

    joint_state_pub_->publish(msg);
    last_update_time_ = current_time;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmJointStateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
