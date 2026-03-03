#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/base_motor.hpp"

class SerialBridge : public rclcpp::Node
{
public:
  SerialBridge() : Node("serial_bridge")
  {
    serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);

    if(serial_fd_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to open /dev/ttyACM0: %s", std::strerror(errno));
    }
    else if(!configureSerial())
    {
      close(serial_fd_);
      serial_fd_ = -1;
    }

    base_sub_ = create_subscription<redburi_msgs::msg::BaseMotor>(
      "/base_motor",
      10,
      [this](redburi_msgs::msg::BaseMotor::SharedPtr msg)
      {
        baseMotorCallback(msg);
      }
    );
  }

  ~SerialBridge()
  {
    if(serial_fd_ >= 0)
    {
      close(serial_fd_);
    }
  }

private:
  rclcpp::Subscription<redburi_msgs::msg::BaseMotor>::SharedPtr base_sub_;
  int serial_fd_{-1};

  bool configureSerial()
  {
    termios tty{};

    if(tcgetattr(serial_fd_, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return false;
    }

    return true;
  }

  void baseMotorCallback(const redburi_msgs::msg::BaseMotor::SharedPtr msg)
  {
    char buffer[128]{};
    int length{};

    RCLCPP_INFO(
      get_logger(),
      "base_motor steer=%.2f f=%.2f r=%.2f l=%.2f",
      msg->steer_deg,
      msg->motor_f_rpm,
      msg->motor_r_rpm,
      msg->motor_l_rpm
    );

    if(serial_fd_ < 0)
    {
      return;
    }

    length = std::snprintf(
      buffer,
      sizeof(buffer),
      "B,%.2f,%.2f,%.2f,%.2f\n",
      msg->steer_deg,
      msg->motor_f_rpm,
      msg->motor_r_rpm,
      msg->motor_l_rpm
    );

    if(length <= 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to format serial message");
      return;
    }

    if(write(serial_fd_, buffer, static_cast<size_t>(length)) < 0)
    {
      RCLCPP_ERROR(get_logger(), "Serial write failed: %s", std::strerror(errno));
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridge>());
  rclcpp::shutdown();
  return 0;
}
