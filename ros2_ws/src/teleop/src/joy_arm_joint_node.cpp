#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "control_msgs/msg/joint_jog.hpp"
#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/arm_motor.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class JoyArmJointNode : public rclcpp::Node
{
public:
  JoyArmJointNode() : Node("joy_arm_joint_node")
  {
    deadzone_drive_ = declare_parameter<double>("deadzone_drive");
    drive_input_max_ = declare_parameter<double>("drive_input_max");
    max_motor_rpm_ = declare_parameter<double>("max_motor_rpm");
    output_mode_ = declare_parameter<std::string>("output_mode", "arm_motor");
    arm_command_topic_ = declare_parameter<std::string>("arm_command_topic", "/arm_motor");
    joint_command_topic_ = declare_parameter<std::string>(
      "joint_command_topic", "/servo_node/delta_joint_cmds");
    gripper_command_topic_ = declare_parameter<std::string>(
      "gripper_command_topic", "/arm_gripper");

    if (output_mode_ != "arm_motor" && output_mode_ != "joint_jog") {
      RCLCPP_WARN(
        get_logger(),
        "unknown output_mode '%s', falling back to arm_motor",
        output_mode_.c_str());
      output_mode_ = "arm_motor";
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy_arm_joint",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joy_arm_joint_callback(msg);
      }
    );
    arm_pub_ = create_publisher<redburi_msgs::msg::ArmMotor>(arm_command_topic_, 10);
    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(joint_command_topic_, 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float32>(gripper_command_topic_, 10);
  }

private:
  static constexpr double RPM_TO_RAD_PER_SEC = (2.0 * M_PI) / 60.0;
  static constexpr std::array<const char *, 6> JOINT_NAMES{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  const int next_button_{5};
  const int prev_button_{4};
  const int axis_drive_{1};
  double deadzone_drive_{};
  double drive_input_max_{};
  double max_motor_rpm_{};
  std::string output_mode_{};
  std::string arm_command_topic_{};
  std::string joint_command_topic_{};
  std::string gripper_command_topic_{};
  int joint_num_{};
  bool was_next_button_pressed_{};
  bool was_prev_button_pressed_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<redburi_msgs::msg::ArmMotor>::SharedPtr arm_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;

  void publishArmMotor(double motor_rpm)
  {
    redburi_msgs::msg::ArmMotor arm{};

    switch (joint_num_) {
      case 0:
        arm.joint_1_rpm = motor_rpm;
        break;
      case 1:
        arm.joint_2_rpm = motor_rpm;
        break;
      case 2:
        arm.joint_3_rpm = motor_rpm;
        break;
      case 3:
        arm.joint_4_rpm = motor_rpm;
        break;
      case 4:
        arm.joint_5_rpm = motor_rpm;
        break;
      case 5:
        arm.joint_6_rpm = motor_rpm;
        break;
      case 6:
        arm.gripper_rpm = motor_rpm;
        break;
      default:
        break;
    }

    arm_pub_->publish(arm);
  }

  void publishJointJog(double joint_velocity_rad_per_sec, double normalized_gripper_command)
  {
    control_msgs::msg::JointJog jog{};
    std_msgs::msg::Float32 gripper{};

    jog.header.stamp = now();

    if (joint_num_ >= 0 && joint_num_ < static_cast<int>(JOINT_NAMES.size())) {
      jog.joint_names.emplace_back(JOINT_NAMES[static_cast<size_t>(joint_num_)]);
      jog.velocities.emplace_back(joint_velocity_rad_per_sec);
    } else {
      jog.joint_names.assign(JOINT_NAMES.begin(), JOINT_NAMES.end());
      jog.velocities.assign(JOINT_NAMES.size(), 0.0);
      gripper.data = static_cast<float>(normalized_gripper_command);
    }

    joint_pub_->publish(jog);
    gripper_pub_->publish(gripper);
  }

  void publishJointJogHalt()
  {
    publishJointJog(0.0, 0.0);
  }

  void joy_arm_joint_callback(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const size_t max_button_idx = static_cast<size_t>(
      std::max(next_button_, prev_button_));
    const size_t axis_idx = static_cast<size_t>(axis_drive_);
    double drive{};

    if(msg->buttons.size() <= max_button_idx || msg->axes.size() <= axis_idx)
    {
      if (output_mode_ == "joint_jog") {
        publishJointJogHalt();
      } else {
        publishArmMotor(0.0);
      }
      return;
    }

    const bool is_next_button_pressed = msg->buttons[next_button_];
    const bool is_prev_button_pressed = msg->buttons[prev_button_];

    if(drive_input_max_ != 0)
    {
      drive = msg->axes[axis_drive_] / drive_input_max_;
    }

    drive = std::clamp(drive, -1.0, 1.0);
    if(std::fabs(drive) < deadzone_drive_) drive = 0.0;
    double motor_rpm = drive * max_motor_rpm_;

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

    if (output_mode_ == "joint_jog") {
      const double joint_velocity_rad_per_sec = motor_rpm * RPM_TO_RAD_PER_SEC;
      const double gripper_command = (joint_num_ == 6) ? drive : 0.0;
      publishJointJog(joint_velocity_rad_per_sec, gripper_command);
      return;
    }

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
