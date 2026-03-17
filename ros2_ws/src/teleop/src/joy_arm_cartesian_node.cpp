#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"

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
    gripper_lower_rad_ = declare_parameter<double>("gripper_lower_deg") * DEG_TO_RAD;
    gripper_upper_rad_ = declare_parameter<double>("gripper_upper_deg") * DEG_TO_RAD;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyArmCartesianCallback(msg);
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
    gripper_state_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/gripper_state",
      10,
      [this](std_msgs::msg::Float32::SharedPtr msg)
      {
        gripperStateCallback(msg);
      }
    );
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    joint_6_pub_ = create_publisher<std_msgs::msg::Int8>("/arm_joint_6", 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float32>("/arm_gripper", 10);
  }

private:
  static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
  static constexpr const char * SERVO_COMMAND_FRAME = "link_6";
  int axis_x_{4};
  int axis_y_{0};
  int axis_z_{1};
  int axis_gripper_open_{5};
  int axis_gripper_close_{2};
  int button_joint_6_right_{5};
  int button_joint_6_left_{4};
  double deadzone_x_{};
  double deadzone_y_{};
  double deadzone_z_{};
  double input_max_x_{};
  double input_max_y_{};
  double input_max_z_{};
  double max_linear_x_{};
  double max_linear_y_{};
  double max_linear_z_{};
  double gripper_lower_rad_{};
  double gripper_upper_rad_{};
  double current_gripper_position_{};
  bool has_gripper_position_{false};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr joint_6_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
  uint8_t control_mode_{0};

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

  void gripperStateCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_gripper_position_ = static_cast<double>(msg->data);
    has_gripper_position_ = true;
  }

  double applyGripperLimit(double gripper_command)
  {
    if (gripper_command == 0.0)
    {
      return 0.0;
    }

    if (!has_gripper_position_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "gripper state is unavailable, halting gripper control for safety");
      return 0.0;
    }

    if (gripper_command > 0.0 && current_gripper_position_ >= gripper_upper_rad_)
    {
      return 0.0;
    }

    if (gripper_command < 0.0 && current_gripper_position_ <= gripper_lower_rad_)
    {
      return 0.0;
    }

    return gripper_command;
  }

  void joyArmCartesianCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped twist{};
    std_msgs::msg::Int8 joint_6{};
    std_msgs::msg::Float32 gripper{};

    const size_t max_axis_idx = static_cast<size_t>(
      std::max({axis_x_, axis_y_, axis_z_, axis_gripper_open_, axis_gripper_close_}));
    const size_t max_button_idx = static_cast<size_t>(
      std::max(button_joint_6_right_, button_joint_6_left_));

    if(control_mode_ != 2)
    {
      twist.header.stamp = now();
      twist.header.frame_id = SERVO_COMMAND_FRAME;
      twist_pub_->publish(twist);
      joint_6_pub_->publish(joint_6);
      gripper_pub_->publish(gripper);
      return;
    }

    if(msg->axes.size() <= max_axis_idx || msg->buttons.size() <= max_button_idx)
    {
      twist.header.stamp = now();
      twist.header.frame_id = SERVO_COMMAND_FRAME;
      twist_pub_->publish(twist);
      joint_6_pub_->publish(joint_6);
      gripper_pub_->publish(gripper);
      return;
    }

    twist.header.stamp = now();
    twist.header.frame_id = SERVO_COMMAND_FRAME;
    twist.twist.linear.x = -scaleAxis(
      msg->axes[axis_x_], input_max_x_, deadzone_x_, max_linear_x_);
    twist.twist.linear.y = scaleAxis(
      msg->axes[axis_y_], input_max_y_, deadzone_y_, max_linear_y_);
    twist.twist.linear.z = scaleAxis(
      msg->axes[axis_z_], input_max_z_, deadzone_z_, max_linear_z_);

    twist_pub_->publish(twist);

    const int right = msg->buttons[button_joint_6_right_];
    const int left = msg->buttons[button_joint_6_left_];
    joint_6.data = static_cast<int8_t>(left - right);
    joint_6_pub_->publish(joint_6);

    const double gripper_open = scaleAxis(
      (1.0 - msg->axes[axis_gripper_open_]) / 2.0,
      input_max_x_,
      deadzone_x_,
      1.0);

    const double gripper_close = scaleAxis(
      (1.0 - msg->axes[axis_gripper_close_]) / 2.0,
      input_max_x_,
      deadzone_x_,
      1.0);

    gripper.data = static_cast<float>(applyGripperLimit(gripper_open - gripper_close));
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
