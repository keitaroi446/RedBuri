#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "redburi_msgs/msg/base_motor.hpp"

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
    wheelbase_m_ = declare_parameter<double>("wheelbase_m");
    tread_m_ = declare_parameter<double>("tread_m");
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
    base_pub_ = create_publisher<redburi_msgs::msg::BaseMotor>("/base_motor", 10);
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
  double wheelbase_m_{};
  double tread_m_{};
  double max_spin_rpm_{};
  double max_motor_rpm_{};
  double max_steer_deg_{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<redburi_msgs::msg::BaseMotor>::SharedPtr base_pub_;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    redburi_msgs::msg::BaseMotor base;
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

    forward = (1.0 - msg->axes[axis_forward_]) / (2.0 * forward_input_max_);
    backward = (1.0 - msg->axes[axis_backward_]) / (2.0 * backward_input_max_);
    steer = msg->axes[axis_steer_] / steer_input_max_;
    spin = msg->axes[axis_spin_] / spin_input_max_;

    forward = std::clamp(forward, 0.0, 1.0);
    backward = std::clamp(backward, 0.0, 1.0);
    steer = std::clamp(steer, -1.0, 1.0);
    spin = std::clamp(spin, -1.0, 1.0);

    if(forward > deadzone_forward_)
    {
      drive += forward;
    }
    
    if(backward > deadzone_backward_)
    {
      drive -= backward;
    }

    if(std::fabs(steer) <= deadzone_steer_)
    {
      steer = 0.0;
    }
    
    if(std::fabs(spin) <= deadzone_spin_)
    {
      spin = 0.0;
    }

    if(spin != 0)
    {
      base.steer_deg = 90.0;
      base.motor_f_rpm = max_spin_rpm_ * spin;
      base.motor_r_rpm = base.motor_f_rpm * (tread_m_ / 2.0) / wheelbase_m_;
      base.motor_l_rpm = -base.motor_r_rpm;
    }
    else if(drive != 0.0 || steer != 0.0)
    {
      base.steer_deg = max_steer_deg_ * steer;
      double steer_rad = std::fabs(base.steer_deg) * M_PI / 180.0; 
      base.motor_f_rpm = max_motor_rpm_ * drive;

      if(steer == 0.0)
      {
        base.motor_r_rpm = base.motor_f_rpm;
        base.motor_l_rpm = base.motor_f_rpm;  
      }
      else
      {
        double outer_rpm = base.motor_f_rpm * (wheelbase_m_ / std::tan(steer_rad) + tread_m_ / 2.0) / (wheelbase_m_ / std::sin(steer_rad));
        double inner_rpm = base.motor_f_rpm * (wheelbase_m_ / std::tan(steer_rad) - tread_m_ / 2.0) / (wheelbase_m_ / std::sin(steer_rad));

        if(steer > 0.0)
        {
          base.motor_r_rpm = outer_rpm;
          base.motor_l_rpm = inner_rpm;
        }
        else
        {
          base.motor_r_rpm = inner_rpm;
          base.motor_l_rpm = outer_rpm;
        }
      }
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