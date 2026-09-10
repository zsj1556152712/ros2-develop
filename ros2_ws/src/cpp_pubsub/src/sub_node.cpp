#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 订阅节点类
class SubNode : public rclcpp::Node {
public:
    SubNode()
        : Node("sub_node") {
        // 订阅 "topic" 频道，收到消息就触发回调
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&SubNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        // 任务要求：收到数据后用 Logger 打印
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubNode>());
    rclcpp::shutdown();
    return 0;
}
