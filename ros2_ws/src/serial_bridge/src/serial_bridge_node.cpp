#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_bridge/serial.hpp"

class SerialBridgeNode : public rclcpp::Node
{
public:
    SerialBridgeNode() : Node("serial_bridge_node")
    {
        if(!serial_.isReady()) return;

        publisher_ = this->create_publisher<std_msgs::msg::String>("stm32_log", 10);
        running_ = true;

        try {
            read_thread_ = std::thread(&SerialBridgeNode::readLoop, this);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "thread start failed: %s", e.what());
            return;
        }

        node_ready_ = true;
    }

    ~SerialBridgeNode()
    {
        running_ = false;
        if(read_thread_.joinable())
        {
            read_thread_.join();
        }
    }

    bool isReady() const
    {
        return node_ready_;
    }

private:
    void readLoop()
    {
        while(running_)
        {
            ssize_t n = serial_.readSerial(buf_, sizeof(buf_));
        }
    }

    Serial serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    uint8_t buf_[1024]{};
    std::atomic<bool> running_{false};
    std::thread read_thread_;
    bool node_ready_{false};
};