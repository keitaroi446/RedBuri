#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_bridge/serial.hpp"

class SerialBridgeNode : public rclcpp::Node
{
public:
    SerialBridgeNode() : Node("serial_bridge_node"),
        serial_(declare_parameter<std::string>("port", "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0670FFB38315939314036931-if02"),
                declare_parameter<int>("baud", 115200))
    {
        const auto rx_topic = declare_parameter<std::string>("rx_topic", "stm32_log");
        const auto tx_topic = declare_parameter<std::string>("tx_topic", "motor_cmd");

        if(!serial_.isReady()) {
            RCLCPP_ERROR(get_logger(), "serial not ready (port/baud mismatch?)");
            return;
        }

        publisher_ = this->create_publisher<std_msgs::msg::String>(rx_topic, 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            tx_topic, 10,
            [this](std_msgs::msg::String::UniquePtr msg) {
                if(!running_) return;
                std::string out = msg->data;
                out.erase(std::remove_if(out.begin(), out.end(),
                                         [](unsigned char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }),
                          out.end());
                if(out.empty()) return;
                out.push_back('\n');
                if(!serial_.writeSerial(out)) {
                    RCLCPP_WARN(get_logger(), "serial write failed");
                }
            });
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
            const ssize_t n = serial_.readSerial(buf_, sizeof(buf_));
            if(n <= 0) continue;

            read_buf_.append(reinterpret_cast<const char*>(buf_), static_cast<size_t>(n));
            size_t pos = 0;
            while((pos = read_buf_.find('\n')) != std::string::npos)
            {
                std::string line = read_buf_.substr(0, pos);
                read_buf_.erase(0, pos + 1);
                if(!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                if(line.empty()) continue;
                std_msgs::msg::String msg;
                msg.data = line;
                publisher_->publish(msg);
            }
        }
    }

    Serial serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    uint8_t buf_[1024]{};
    std::string read_buf_;
    std::atomic<bool> running_{false};
    std::thread read_thread_;
    bool node_ready_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialBridgeNode>();
    if(!node->isReady()) {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
