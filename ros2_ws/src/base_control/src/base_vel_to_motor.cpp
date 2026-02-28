#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/base_vel.hpp"

class CmdVelToMotor : public rclcpp::Node
{
public:
  CmdVelToMotor() : Node("base_vel_to_motor")
  {
    cmd_sub_ = create_subscription<redburi_msgs::msg::BaseVel>(
      "/base_vel",
      10,
      [this](redburi_msgs::msg::BaseVel::SharedPtr msg)
      {
        cmdVelCallback(msg);
      }
    );
  }

private:
  static constexpr double R = 0.11;      // タイヤ半径[m]
  static constexpr double L = 0.930535;  // ホイールベース[m]
  static constexpr double T = 0.675026;  // トレッド[m]
  static constexpr double MAX_STEER_DEG = 30.0;
  rclcpp::Subscription<redburi_msgs::msg::BaseVel>::SharedPtr cmd_sub_;
  
  void cmdVelCallback(const redburi_msgs::msg::BaseVel::SharedPtr msg)
  {
    double v{msg->linear};
    double w_turn{msg->angular_turn};
    double w_rot{msg->angular_rot};
    double steer_deg{};
    double motor_f_rpm{};
    double motor_r_rpm{};
    double motor_l_rpm{};
    
    if(w_rot != 0.0)
    {
      steer_deg = 90.0;
      motor_f_rpm = 30.0 * (w_rot * L) / (M_PI * R);
      motor_r_rpm = -30.0 * (w_rot * T / 2) / (M_PI * R);
      motor_l_rpm = 30.0 * (w_rot * T / 2) / (M_PI * R);
    }
    else if(v != 0.0 || w_turn != 0.0)
    {
      if(v != 0.0)
      {
        steer_deg = atan(L * w_turn / fabs(v)) * 180.0 / M_PI;
        if(v < 0.0) steer_deg *= -1;
        steer_deg = std::clamp(steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG);
      }

      motor_f_rpm = 30.0 * v / (M_PI * R);
      motor_r_rpm = 30.0 * (v + w_turn * T / 2) / (M_PI * R);
      motor_l_rpm = 30.0 * (v - w_turn * T / 2) / (M_PI * R);
    }

    RCLCPP_INFO(
      get_logger(),
      "base_vel v=%.3f turn=%.3f rot=%.3f steer=%.2f f=%.2f l=%.2f r=%.2f",
      v, w_turn, w_rot, steer_deg, motor_f_rpm, motor_l_rpm, motor_r_rpm
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToMotor>());
  rclcpp::shutdown();
  return 0;
}
