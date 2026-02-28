#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "redburi_msgs/msg/base_command.hpp"

class JoyToBaseCmd : public rclcpp::Node
{
public:
  JoyToBaseCmd() : Node("joy_to_base_cmd")
  {
    axis_forward_ = declare_parameter<int>("axis_forward");
    axis_backward_ = declare_parameter<int>("axis_backward");
    axis_steer_ = declare_parameter<int>("axis_steer");
    axis_spin_ = declare_parameter<int>("axis_spin");
    scale_forward_ = declare_parameter<double>("scale_forward");
    scale_backward_ = declare_parameter<double>("scale_backward");
    scale_steer_ = declare_parameter<double>("scale_steer");
    scale_spin_ = declare_parameter<double>("scale_spin");
    deadzone_forward_ = declare_parameter<double>("deadzone_forward");
    deadzone_backward_ = declare_parameter<double>("deadzone_backward");
    deadzone_steer_ = declare_parameter<double>("deadzone_steer");
    deadzone_spin_ = declare_parameter<double>("deadzone_spin");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyCallback(msg);
      }
    );
    cmd_pub_ = create_publisher<redburi_msgs::msg::BaseCommand>("/base_cmd", 10);
  }

private:
  int axis_forward_{};
  int axis_backward_{};
  int axis_steer_{};
  int axis_spin_{};
  double scale_forward_{};
  double scale_backward_{};
  double scale_steer_{};
  double scale_spin_{};
  double deadzone_forward_{};
  double deadzone_backward_{};
  double deadzone_steer_{};
  double deadzone_spin_{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<redburi_msgs::msg::BaseCommand>::SharedPtr cmd_pub_;

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
      cmd_pub_->publish(base);
      return;
    }

    forward = (1.0 - msg->axes[axis_forward_]) / 2.0;
    backward = (1.0 - msg->axes[axis_backward_]) / 2.0;
    steer = msg->axes[axis_steer_];
    spin = msg->axes[axis_spin_];

    if(std::fabs(forward) > deadzone_forward_)
    {
      drive += forward * scale_forward_;
    }
    if(std::fabs(backward) > deadzone_backward_)
    {
      drive -= backward * scale_backward_;
    }

    if(std::fabs(spin) > deadzone_spin_)
    {
      base.spin = spin * scale_spin_;
    }
    else
    {
      if(std::fabs(drive) > 0.0)
      {
        base.drive = drive;
      }

      if(std::fabs(steer) > deadzone_steer_)
      {
        base.steer = steer * scale_steer_;
      }
    }
    
    cmd_pub_->publish(base);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToBaseCmd>());
  rclcpp::shutdown();
  return 0;
}
