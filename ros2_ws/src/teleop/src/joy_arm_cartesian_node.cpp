#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class JoyArmCartesianNode : public rclcpp::Node
{
public:
  JoyArmCartesianNode() : Node("joy_arm_cartesian_node")
  {
    deadzone_x_ = declare_parameter<double>("deadzone_x");
    deadzone_y_ = declare_parameter<double>("deadzone_y");
    deadzone_z_ = declare_parameter<double>("deadzone_z");
    input_max_x_ = declare_parameter<double>("input_max_x");
    input_max_y_ = declare_parameter<double>("input_max_y");
    input_max_z_ = declare_parameter<double>("input_max_z");
    max_linear_x_ = declare_parameter<double>("max_linear_x");
    max_linear_y_ = declare_parameter<double>("max_linear_y");
    max_linear_z_ = declare_parameter<double>("max_linear_z");
    servo_command_frame_ = declare_parameter<std::string>("servo_command_frame", "base_mount");
    servo_twist_topic_ = declare_parameter<std::string>(
      "servo_twist_topic", "/servo_node/delta_twist_cmds");
    gripper_command_topic_ = declare_parameter<std::string>("gripper_command_topic", "/arm_gripper");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy_arm_cartesian",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyArmCartesianCallback(msg);
      }
    );
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(servo_twist_topic_, 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float32>(gripper_command_topic_, 10);
  }

private:
  int axis_x_{0};
  int axis_y_{1};
  int axis_z_{4};
  int axis_gripper_open_{5};
  int axis_gripper_close_{2};
  double deadzone_x_{};
  double deadzone_y_{};
  double deadzone_z_{};
  double input_max_x_{};
  double input_max_y_{};
  double input_max_z_{};
  double max_linear_x_{};
  double max_linear_y_{};
  double max_linear_z_{};
  std::string servo_command_frame_{};
  std::string servo_twist_topic_{};
  std::string gripper_command_topic_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;

  double scaleAxis(double input, double input_max, double deadzone, double max_linear) const
  {
    if(input_max == 0.0)
    {
      return 0.0;
    }

    double normalized = std::clamp(input / input_max, -1.0, 1.0);

    if(std::fabs(normalized) < deadzone)
    {
      return 0.0;
    }

    return normalized * max_linear;
  }

  void joyArmCartesianCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped twist{};
    std_msgs::msg::Float32 gripper{};
    const size_t max_axis_idx = static_cast<size_t>(
      std::max({axis_x_, axis_y_, axis_z_, axis_gripper_open_, axis_gripper_close_}));

    if(msg->axes.size() <= max_axis_idx)
    {
      twist.header.stamp = now();
      twist.header.frame_id = servo_command_frame_;
      twist_pub_->publish(twist);
      gripper_pub_->publish(gripper);
      return;
    }

    twist.header.stamp = now();
    twist.header.frame_id = servo_command_frame_;
    twist.twist.linear.x = -scaleAxis(
      msg->axes[axis_x_], input_max_x_, deadzone_x_, max_linear_x_);
    twist.twist.linear.y = scaleAxis(
      msg->axes[axis_y_], input_max_y_, deadzone_y_, max_linear_y_);
    twist.twist.linear.z = scaleAxis(
      msg->axes[axis_z_], input_max_z_, deadzone_z_, max_linear_z_);

    twist_pub_->publish(twist);

    const double gripper_open = scaleAxis(
      (1.0 - msg->axes[axis_gripper_open_]) / 2.0,
      input_max_x_,
      deadzone_x_,
      max_linear_x_);

    const double gripper_close = scaleAxis(
      (1.0 - msg->axes[axis_gripper_close_]) / 2.0,
      input_max_x_,
      deadzone_x_,
      max_linear_x_);

    gripper.data = gripper_open - gripper_close;
    gripper_pub_->publish(gripper);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyArmCartesianNode>());
  rclcpp::shutdown();
  return 0;
}
