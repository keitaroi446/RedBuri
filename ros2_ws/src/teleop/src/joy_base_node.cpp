#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "redburi_msgs/msg/base_command.hpp"

class JoyBaseNode : public rclcpp::Node
{
public:
  JoyBaseNode() : Node("joy_base_node")
  {
    forward_input_max_ = declare_parameter<double>("forward_input_max");
    backward_input_max_ = declare_parameter<double>("backward_input_max");
    steer_input_max_ = declare_parameter<double>("steer_input_max");
    spin_input_max_ = declare_parameter<double>("spin_input_max");
    deadzone_forward_ = declare_parameter<double>("deadzone_forward");
    deadzone_backward_ = declare_parameter<double>("deadzone_backward");
    deadzone_steer_ = declare_parameter<double>("deadzone_steer");
    deadzone_spin_ = declare_parameter<double>("deadzone_spin");
    max_spin_rpm_ = declare_parameter<double>("max_spin_rpm");
    max_motor_rpm_ = declare_parameter<double>("max_motor_rpm");
    max_steer_deg_ = declare_parameter<double>("max_steer_deg");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy_base",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyCallback(msg);
      }
    );
    base_pub_ = create_publisher<redburi_msgs::msg::BaseCommand>("/base_cmd", 10);
  }

private:
  int axis_forward_{5};
  int axis_backward_{2};
  int axis_steer_{0};
  int axis_spin_{3};
  double forward_input_max_{};
  double backward_input_max_{};
  double steer_input_max_{};
  double spin_input_max_{};
  double deadzone_forward_{};
  double deadzone_backward_{};
  double deadzone_steer_{};
  double deadzone_spin_{};
  double max_spin_rpm_{};
  double max_motor_rpm_{};
  double max_steer_deg_{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<redburi_msgs::msg::BaseCommand>::SharedPtr base_pub_;

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
    double spin{};

    const size_t max_idx = static_cast<size_t>(
      std::max({axis_forward_, axis_backward_, axis_steer_, axis_spin_}));

    if(msg->axes.size() <= max_idx)
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
    steer = scaleAxis(
      msg->axes[axis_steer_],
      steer_input_max_,
      deadzone_steer_,
      1.0);
    spin = scaleAxis(
      msg->axes[axis_spin_],
      spin_input_max_,
      deadzone_spin_,
      1.0);

    drive = forward - backward;

    if(spin != 0)
    {
      base.target_steer_deg = 90.0;
      base.motor_rpm = max_spin_rpm_ * spin;
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
