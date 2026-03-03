#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "redburi_msgs/msg/arm_joint.hpp"

class JoyArmJointNode : public rclcpp::Node
{
public:
  JoyArmJointNode() : Node("joy_arm_joint_node")
  {
    deadzone_drive_ = declare_parameter<double>("deadzone_drive");
    drive_input_max_ = declare_parameter<double>("drive_input_max");
    max_motor_rpm_ = declare_parameter<double>("max_motor_rpm");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy_arm_joint",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joy_arm_joint_callback(msg);
      }
    );
    arm_pub_ = create_publisher<redburi_msgs::msg::ArmJoint>("/arm_joint", 10);
  }

private:
  const int next_button_{5};
  const int prev_button_{4};
  const int axis_drive_{1};
  double deadzone_drive_{};
  double drive_input_max_{};
  double max_motor_rpm_{};
  int joint_num_{};
  bool was_next_button_pressed_{};
  bool was_prev_button_pressed_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<redburi_msgs::msg::ArmJoint>::SharedPtr arm_pub_;

  void joy_arm_joint_callback(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    redburi_msgs::msg::ArmJoint arm;
    const size_t max_button_idx = static_cast<size_t>(
      std::max(next_button_, prev_button_));
    const size_t axis_idx = static_cast<size_t>(axis_drive_);
    double drive{};

    if(msg->buttons.size() <= max_button_idx || msg->axes.size() <= axis_idx)
    {
      arm_pub_->publish(arm);
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

    switch(joint_num_)
    {
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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyArmJointNode>());
  rclcpp::shutdown();
  return 0;
}
