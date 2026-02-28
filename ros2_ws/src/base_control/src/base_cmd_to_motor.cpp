#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/base_command.hpp"

class BaseCmdToMotor : public rclcpp::Node
{
public:
  BaseCmdToMotor() : Node("base_cmd_to_motor")
  {
    cmd_sub_ = create_subscription<redburi_msgs::msg::BaseCommand>(
      "/base_cmd",
      10,
      [this](redburi_msgs::msg::BaseCommand::SharedPtr msg)
      {
        baseCmdCallback(msg);
      }
    );
  }

private:
  static constexpr double R = 0.11;      // タイヤ半径[m]
  static constexpr double L = 0.930535;  // ホイールベース[m]
  static constexpr double T = 0.675026;  // トレッド[m]
  static constexpr double MAX_STEER_DEG = 45.0;
  static constexpr double EPS = 1e-4;
  rclcpp::Subscription<redburi_msgs::msg::BaseCommand>::SharedPtr cmd_sub_;
  
  void baseCmdCallback(const redburi_msgs::msg::BaseCommand::SharedPtr msg)
  {
    double drive{msg->drive};
    double steer{msg->steer};
    double spin{msg->spin};
    double steer_deg{};
    double steer_rad{};
    double turn_rate{};
    double motor_f_rpm{};
    double motor_r_rpm{};
    double motor_l_rpm{};
    const double k = 30.0 / (M_PI * R);

    if(std::fabs(spin) > EPS)
    {
      steer_deg = 90.0;
      motor_f_rpm = k * (spin * L);
      motor_r_rpm = -k * (spin * T / 2.0);
      motor_l_rpm = k * (spin * T / 2.0);
    }
    else
    {
      steer_deg = steer * MAX_STEER_DEG;
      steer_deg = std::clamp(steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG);
      steer_rad = steer_deg * M_PI / 180.0;
      motor_f_rpm = k * drive;

      if(std::fabs(drive) > EPS)
      {
        turn_rate = drive * std::tan(steer_rad) / L;
        motor_r_rpm = k * (drive + turn_rate * T / 2.0);
        motor_l_rpm = k * (drive - turn_rate * T / 2.0);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "base_cmd drive=%.3f steer=%.3f spin=%.3f steer_deg=%.2f f=%.2f l=%.2f r=%.2f",
      drive, steer, spin, steer_deg, motor_f_rpm, motor_l_rpm, motor_r_rpm
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseCmdToMotor>());
  rclcpp::shutdown();
  return 0;
}
