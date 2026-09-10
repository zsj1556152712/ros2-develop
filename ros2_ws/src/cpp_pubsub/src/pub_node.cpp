#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// 发布节点类
class PubNode : public rclcpp::Node {
public:
    PubNode()
        : Node("pub_node")
        , count_(0) {
        // 创建发布者：往 "topic" 这个频道发消息，队列长度 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // 定时器：每 500 毫秒触发一次回调
        timer_ = this->create_wall_timer(500ms, std::bind(&PubNode::timer_callback, this));
    }

private:
    void timer_callback() {
        // 组装一条字符串消息，内容自定（这里带了个计数）
        auto message = std_msgs::msg::String();
        message.data = "Hello from ROS2, count: " + std::to_string(count_++);
        // 发布
        publisher_->publish(message);
        // 发布者也打个日志，方便观察
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubNode>());
    rclcpp::shutdown();
    return 0;
}
