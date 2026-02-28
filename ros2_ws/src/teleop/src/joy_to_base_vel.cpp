#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "redburi_msgs/msg/base_vel.hpp"

class JoyToCmdVel : public rclcpp::Node
{
public:
  JoyToCmdVel() : Node("joy_to_base_vel")
  {
    base_enable_button_ = declare_parameter<int>("base_enable_button");
    axis_linear_ = declare_parameter<int>("axis_linear");
    axis_angular_turn_ = declare_parameter<int>("axis_angular_turn");
    axis_angular_rot_ = declare_parameter<int>("axis_angular_rot");
    scale_linear_ = declare_parameter<double>("scale_linear");
    scale_angular_turn_ = declare_parameter<double>("scale_angular_turn");
    scale_angular_rot_ = declare_parameter<double>("scale_angular_rot");
    deadzone_linear_ = declare_parameter<double>("deadzone_linear");
    deadzone_angular_turn_ = declare_parameter<double>("deadzone_angular_turn");
    deadzone_angular_rot_ = declare_parameter<double>("deadzone_angular_rot");

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyCallback(msg);
      }
    );
    cmd_pub_ = create_publisher<redburi_msgs::msg::BaseVel>("/base_vel", 10);
  }

private:
    int base_enable_button_{};
    int axis_linear_{};
    int axis_angular_turn_{};
    int axis_angular_rot_{};
    double scale_linear_{};
    double scale_angular_turn_{};
    double scale_angular_rot_{};
    double deadzone_linear_{};
    double deadzone_angular_turn_{};
    double deadzone_angular_rot_{};

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<redburi_msgs::msg::BaseVel>::SharedPtr cmd_pub_;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      redburi_msgs::msg::BaseVel base;

      if(msg->buttons.size() > static_cast<size_t>(base_enable_button_) &&
         msg->buttons[base_enable_button_] == 1)
      {
        double lin = 0.0;
        double ang_turn = 0.0;
        double ang_rot = 0.0;

        const size_t max_idx = static_cast<size_t>(
          std::max({axis_linear_, axis_angular_turn_, axis_angular_rot_}));

        if(msg->axes.size() <= max_idx)
        {
          cmd_pub_->publish(redburi_msgs::msg::BaseVel());
          return;
        }

        lin = msg->axes[axis_linear_];
        ang_turn = msg->axes[axis_angular_turn_];
        ang_rot = msg->axes[axis_angular_rot_];

        if(std::fabs(ang_rot) > deadzone_angular_rot_)
        {
          base.linear = 0.0;
          base.angular_rot = ang_rot * scale_angular_rot_;
        }
        else
        {
          if(std::fabs(lin) > deadzone_linear_)
          {
            base.linear = lin * scale_linear_;
            if(std::fabs(ang_turn) > deadzone_angular_turn_)
            {
              base.angular_turn = ang_turn * scale_angular_turn_;
              if(lin < 0.0) base.angular_turn *= -1.0;
            }
          }
        }
      }

      cmd_pub_->publish(base);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel>());
  rclcpp::shutdown();
  return 0;
}
