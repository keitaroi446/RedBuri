#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToCmdVel : public rclcpp::Node
{
public:
  JoyToCmdVel() : Node("joy_to_cmd_vel")
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
      std::bind(&JoyToCmdVel::onJoy, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      geometry_msgs::msg::Twist cmd;

      if (msg->buttons.size() > 5 && msg->buttons[5] == 1)
      {
        if (msg->axes.size() > 1) cmd.linear.x = msg->axes[1] * 0.6;
        if (msg->axes.size() > 0) cmd.angular.z = msg->axes[0] * 1.0;
      }

      cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel>());
  rclcpp::shutdown();
  return 0;
}
