#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "redburi_msgs/msg/base_command.hpp"
#include "std_msgs/msg/u_int8.hpp"

class JoyBaseNode : public rclcpp::Node
{
public:
  JoyBaseNode() : Node("joy_base_node")
  {
    forward_input_max_ = declare_parameter<double>("forward_input_max");
    backward_input_max_ = declare_parameter<double>("backward_input_max");
    steer_input_max_ = declare_parameter<double>("steer_input_max");
    deadzone_forward_ = declare_parameter<double>("deadzone_forward");
    deadzone_backward_ = declare_parameter<double>("deadzone_backward");
    deadzone_steer_ = declare_parameter<double>("deadzone_steer");
    spin_rpm_ = declare_parameter<double>("spin_rpm");
    max_motor_rpm_ = declare_parameter<double>("max_motor_rpm");
    max_steer_deg_ = declare_parameter<double>("max_steer_deg");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyCallback(msg);
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
    base_pub_ = create_publisher<redburi_msgs::msg::BaseCommand>("/base_cmd", 10);
  }

private:
  int axis_forward_{5};
  int axis_backward_{2};
  int axis_steer_{0};
  int button_right_spin{5};
  int button_left_spin{4};
  double forward_input_max_{};
  double backward_input_max_{};
  double steer_input_max_{};
  double deadzone_forward_{};
  double deadzone_backward_{};
  double deadzone_steer_{};
  double spin_rpm_{};
  double max_motor_rpm_{};
  double max_steer_deg_{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<redburi_msgs::msg::BaseCommand>::SharedPtr base_pub_;
  uint8_t control_mode_{0};

  double scaleAxis(double input, double input_max, double deadzone, double max_output) const
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

    return normalized * max_output;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    redburi_msgs::msg::BaseCommand base;
    double forward{};
    double backward{};
    double drive{};
    double steer{};
    int right_spin{};
    int left_spin{};

    if(control_mode_ != 1)
    {
      base_pub_->publish(base);
      return;
    }

    const size_t max_axis_idx = static_cast<size_t>(
      std::max({axis_forward_, axis_backward_, axis_steer_}));
    const size_t max_button_idx = static_cast<size_t>(
      std::max(button_right_spin, button_left_spin));

    if(msg->axes.size() <= max_axis_idx || msg->buttons.size() <= max_button_idx)
    {
      base_pub_->publish(base);
      return;
    }

    forward = scaleAxis(
      (1.0 - msg->axes[axis_forward_]) / 2.0,
      forward_input_max_,
      deadzone_forward_,
      1.0);
    backward = scaleAxis(
      (1.0 - msg->axes[axis_backward_]) / 2.0,
      backward_input_max_,
      deadzone_backward_,
      1.0);
    drive = forward - backward;

    steer = scaleAxis(
      msg->axes[axis_steer_],
      steer_input_max_,
      deadzone_steer_,
      1.0);

    right_spin = msg->buttons[button_right_spin];
    left_spin = msg->buttons[button_left_spin];

    if(right_spin != left_spin)
    {
      if(right_spin != 0)
      {
        base.target_steer_deg = -90.0;
        base.motor_rpm = spin_rpm_;
      }
      else
      {
        base.target_steer_deg = 90.0;
        base.motor_rpm = spin_rpm_;
      }
    }
    else if(drive != 0.0 || steer != 0.0)
    {
      base.target_steer_deg = max_steer_deg_ * steer;
      base.motor_rpm = max_motor_rpm_ * drive;
    }

    base_pub_->publish(base);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyBaseNode>());
  rclcpp::shutdown();
  return 0;
}
