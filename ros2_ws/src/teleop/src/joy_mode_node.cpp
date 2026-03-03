#include <algorithm>
#include <memory>
#include <fstream>
#include <filesystem>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyModeNode : public rclcpp::Node
{
public:
  JoyModeNode() : Node("joy_mode_node")
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg)
      {
        joyCallback(msg);
      }
    );
    joy_base_pub_ = create_publisher<sensor_msgs::msg::Joy>("/joy_base", 10);
    joy_arm_joint_pub_ = create_publisher<sensor_msgs::msg::Joy>("/joy_arm_joint", 10);
    joy_arm_cartesian_pub_ = create_publisher<sensor_msgs::msg::Joy>("/joy_arm_cartesian", 10);
    
    detectLedPaths();
    setLedColor(0, 0, 255);
  }

private:
  int disabled_mode_button_{0}; 
  int base_mode_button_{1};
  int arm_cartesian_mode_button_{2};
  int arm_joint_mode_button_{3};
  int led_max_brightness_{255};
  std::string led_red_path_{};
  std::string led_green_path_{};
  std::string led_blue_path_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_base_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_arm_joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_arm_cartesian_pub_;

  enum class ControlMode
  {
    Disabled,
    Base,
    ArmCartesian,
    ArmJoint
  };

  ControlMode mode_{ControlMode::Disabled};

  bool detectLedPaths()
  {
    namespace fs = std::filesystem;
    const fs::path leds_dir("/sys/class/leds");

    for(const auto &entry : fs::directory_iterator(leds_dir))
    {
      const std::string name = entry.path().filename().string();

      if(name.rfind("input", 0) != 0)
      {
        continue;
      }

      if(name.size() < 5 || name.substr(name.size() - 4) != ":red")
      {
        continue;
      }

      const std::string prefix = name.substr(0, name.size() - 4);

      const fs::path green = leds_dir / (prefix + ":green");
      const fs::path blue = leds_dir / (prefix + ":blue");

      if(fs::exists(green) && fs::exists(blue))
      {
        led_red_path_ = (leds_dir / (prefix + ":red") / "brightness").string();
        led_green_path_ = (leds_dir / (prefix + ":green") / "brightness").string();
        led_blue_path_ = (leds_dir / (prefix + ":blue") / "brightness").string();
        return true;
      }
    }

    return false;
  }

  bool writeLed(const std::string &path, int value)
  {
    std::ofstream file(path);

    if(!file.is_open())
    {
      return false;
    }

    file << value;
    return !file.fail();
  }

  void setLedColor(int red_value, int green_value, int blue_value)
  {
    red_value = std::clamp(red_value, 0, led_max_brightness_);
    green_value = std::clamp(green_value, 0, led_max_brightness_);
    blue_value = std::clamp(blue_value, 0, led_max_brightness_);

    bool ok =
      writeLed(led_red_path_, red_value) &&
      writeLed(led_green_path_, green_value) &&
      writeLed(led_blue_path_, blue_value);

    if(!ok && detectLedPaths())
    {
      writeLed(led_red_path_, red_value);
      writeLed(led_green_path_, green_value);
      writeLed(led_blue_path_, blue_value);
    }
  }


  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const size_t max_button_idx = static_cast<size_t>(
      std::max({
        disabled_mode_button_,
        base_mode_button_,
        arm_cartesian_mode_button_,
        arm_joint_mode_button_,
      }));

    if(msg->buttons.size() <= max_button_idx)
    {
      return;
    }

    const int disabled_pressed = msg->buttons[disabled_mode_button_];
    const int base_pressed = msg->buttons[base_mode_button_];
    const int arm_cartesian_pressed = msg->buttons[arm_cartesian_mode_button_];
    const int arm_joint_pressed = msg->buttons[arm_joint_mode_button_];

    const int pressed_mode_buttons =
      disabled_pressed +
      base_pressed +
      arm_cartesian_pressed +
      arm_joint_pressed;

    if(pressed_mode_buttons == 1)
    {
      if(disabled_pressed && mode_ != ControlMode::Disabled)
      {
        setLedColor(0, 0, 255);
        mode_ = ControlMode::Disabled;
      }
      else if(base_pressed && mode_ != ControlMode::Base)
      {
        setLedColor(255, 0, 0);
        mode_ = ControlMode::Base;
      }
      else if(arm_cartesian_pressed && mode_ != ControlMode::ArmCartesian)
      {
        setLedColor(0, 255, 0);
        mode_ = ControlMode::ArmCartesian;
      }
      else if(arm_joint_pressed && mode_ != ControlMode::ArmJoint) 
      {
        setLedColor(255, 0, 255);
        mode_ = ControlMode::ArmJoint;
      }
    }

    switch(mode_)
    {
      case ControlMode::Disabled:
        break;

      case ControlMode::Base:
        joy_base_pub_->publish(*msg);
        break;

      case ControlMode::ArmCartesian:
        joy_arm_cartesian_pub_->publish(*msg);
        break;

      case ControlMode::ArmJoint:
        joy_arm_joint_pub_->publish(*msg);
        break;

      default:
        break;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyModeNode>());
  rclcpp::shutdown();
  return 0;
}
