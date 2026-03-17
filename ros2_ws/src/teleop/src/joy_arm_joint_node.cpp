#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <chrono>
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/arm_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "urdf/model.h"

class JoyArmJointNode : public rclcpp::Node
{
public:
  JoyArmJointNode() : Node("joy_arm_joint_node")
  {
    deadzone_drive_ = declare_parameter<double>("deadzone_drive");
    drive_input_max_ = declare_parameter<double>("drive_input_max");
    max_motor_rpm_ = declare_parameter<double>("max_motor_rpm");

    load_joint_limits_from_robot_description();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joy_arm_joint_callback(msg);
      }
    );
    mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/control_mode",
      10,
      [this](std_msgs::msg::UInt8::SharedPtr msg)
      {
        control_mode_ = msg->data;
      }
    );
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg)
      {
        joint_state_callback(msg);
      }
    );
    arm_pub_ = create_publisher<redburi_msgs::msg::ArmCommand>("/arm_cmd", 10);
  }

private:
  static constexpr std::array<const char *, 6> JOINT_NAMES{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  const int next_button_{5};
  const int prev_button_{4};
  const int axis_forward_{5};
  const int axis_backward_{2};
  double deadzone_drive_{};
  double drive_input_max_{};
  double max_motor_rpm_{};
  int joint_num_{};
  bool was_next_button_pressed_{};
  bool was_prev_button_pressed_{};
  std::array<double, JOINT_NAMES.size()> joint_lower_limits_{};
  std::array<double, JOINT_NAMES.size()> joint_upper_limits_{};
  std::array<double, JOINT_NAMES.size()> current_joint_positions_{};
  std::array<bool, JOINT_NAMES.size()> has_joint_positions_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<redburi_msgs::msg::ArmCommand>::SharedPtr arm_pub_;
  uint8_t control_mode_{0};

  double scale_axis(double input, double input_max, double deadzone) const
  {
    if (input_max == 0.0) {
      return 0.0;
    }

    double normalized = std::clamp(input / input_max, -1.0, 1.0);
    if (std::fabs(normalized) < deadzone) {
      return 0.0;
    }

    return normalized;
  }

  void load_joint_limits_from_urdf(const std::string & robot_description)
  {
    if (robot_description.empty()) {
      throw std::runtime_error("robot_description is empty");
    }

    urdf::Model model;
    if (!model.initString(robot_description)) {
      throw std::runtime_error("failed to parse robot_description");
    }

    for (size_t joint_idx = 0; joint_idx < JOINT_NAMES.size(); ++joint_idx) {
      const auto joint = model.getJoint(JOINT_NAMES[joint_idx]);
      if (!joint || !joint->limits) {
        throw std::runtime_error(
          std::string("joint limit not found for ") + JOINT_NAMES[joint_idx]);
      }

      joint_lower_limits_[joint_idx] = joint->limits->lower;
      joint_upper_limits_[joint_idx] = joint->limits->upper;
    }
  }

  void load_joint_limits_from_robot_description()
  {
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
    if (!param_client->wait_for_service(std::chrono::seconds(2))) {
      throw std::runtime_error("robot_state_publisher parameter service is unavailable");
    }

    const auto robot_description =
      param_client->get_parameter<std::string>("robot_description", "");
    load_joint_limits_from_urdf(robot_description);
  }

  void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const size_t num_joints = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < num_joints; ++i) {
      for (size_t joint_idx = 0; joint_idx < JOINT_NAMES.size(); ++joint_idx) {
        if (msg->name[i] == JOINT_NAMES[joint_idx]) {
          current_joint_positions_[joint_idx] = msg->position[i];
          has_joint_positions_[joint_idx] = true;
          break;
        }
      }
    }
  }

  double apply_joint_limit(double motor_rpm)
  {
    if (joint_num_ < 0 || joint_num_ >= static_cast<int>(JOINT_NAMES.size()) || motor_rpm == 0.0) {
      return motor_rpm;
    }

    const size_t joint_idx = static_cast<size_t>(joint_num_);
    if (!has_joint_positions_[joint_idx]) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "joint state for %s is unavailable, halting joint control for safety",
        JOINT_NAMES[joint_idx]);
      return 0.0;
    }

    const double lower_limit = joint_lower_limits_[joint_idx];
    const double upper_limit = joint_upper_limits_[joint_idx];
    const double current_position = current_joint_positions_[joint_idx];

    if (motor_rpm > 0.0 && current_position >= upper_limit) {
      return 0.0;
    }

    if (motor_rpm < 0.0 && current_position <= lower_limit) {
      return 0.0;
    }

    return motor_rpm;
  }

  void publishArmMotor(double motor_rpm)
  {
    redburi_msgs::msg::ArmCommand arm_command{};

    switch (joint_num_) {
      case 0:
        arm_command.joint_1_rpm = motor_rpm;
        break;
      case 1:
        arm_command.joint_2_rpm = motor_rpm;
        break;
      case 2:
        arm_command.joint_3_rpm = motor_rpm;
        break;
      case 3:
        arm_command.joint_4_rpm = motor_rpm;
        break;
      case 4:
        arm_command.joint_5_rpm = motor_rpm;
        break;
      case 5:
        arm_command.joint_6_rpm = motor_rpm;
        break;
      case 6:
        arm_command.gripper_rpm = motor_rpm;
        break;
      default:
        break;
    }

    arm_pub_->publish(arm_command);
  }

  void joy_arm_joint_callback(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const size_t max_button_idx = static_cast<size_t>(
      std::max(next_button_, prev_button_));
    const size_t max_axis_idx = static_cast<size_t>(std::max(axis_forward_, axis_backward_));
    double forward{};
    double backward{};
    double drive{};
    
    if(control_mode_ != 3)
    {
      return;
    }

    if(msg->buttons.size() <= max_button_idx || msg->axes.size() <= max_axis_idx)
    {
      publishArmMotor(0.0);
      return;
    }

    const bool is_next_button_pressed = msg->buttons[next_button_];
    const bool is_prev_button_pressed = msg->buttons[prev_button_];

    forward = scale_axis((1.0 - msg->axes[axis_forward_]) / 2.0, drive_input_max_, deadzone_drive_);
    backward = scale_axis((1.0 - msg->axes[axis_backward_]) / 2.0, drive_input_max_, deadzone_drive_);
    drive = forward - backward;

    double motor_rpm = apply_joint_limit(drive * max_motor_rpm_);

    if(is_next_button_pressed && !was_next_button_pressed_)
    {
      joint_num_++;
      if(joint_num_ > 6)
      {
        joint_num_ = 0;
      }
    }

    if(is_prev_button_pressed && !was_prev_button_pressed_)
    {
      joint_num_--;
      if(joint_num_ < 0)
      {
        joint_num_ = 6;
      }
    }

    was_next_button_pressed_ = is_next_button_pressed;
    was_prev_button_pressed_ = is_prev_button_pressed;

    publishArmMotor(motor_rpm);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyArmJointNode>());
  rclcpp::shutdown();
  return 0;
}
