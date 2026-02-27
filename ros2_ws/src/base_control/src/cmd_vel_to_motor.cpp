#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelToMotor : public rclcpp::Node
{
public:
  CmdVelToMotor() : Node("cmd_vel_to_motor")
  {
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg)
      {
        cmdVelCallback(msg);
      }
    );
  }

private:
  static constexpr double R = 0.11;      // タイヤ半径[m]
  static constexpr double L = 0.930535;  // ホイールベース[m]
  static constexpr double T = 0.675026;  // トレッド[m]
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v = msg->linear.x;
    double w = msg->angular.z;
    double steer_deg = 0.0;
    double motor_f_rpm = 0.0;
    double motor_r_rpm = 0.0;
    double motor_l_rpm = 0.0;
    
    if(v != 0.0)
    {
      steer_deg = atan(L * w / fabs(v)) * 180.0 / M_PI;
      if(v < 0.0) steer_deg *= -1;
      motor_f_rpm = 30.0 * v / (M_PI * R);
      motor_r_rpm = 30.0 * (v + w * T / 2) / (M_PI * R);
      motor_l_rpm = 30.0 * (v - w * T / 2) / (M_PI * R);
    }
    else if(w != 0.0)
    {
      steer_deg = 90.0;
      motor_f_rpm = 30.0 * (w * L) / (M_PI * R);
      motor_r_rpm = -30.0 * (w * T / 2) / (M_PI * R);
      motor_l_rpm = 30.0 * (w * T / 2) / (M_PI * R);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToMotor>());
  rclcpp::shutdown();
  return 0;
}

