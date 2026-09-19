#include <algorithm>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>

class FilterNode : public rclcpp::Node {
public:
    FilterNode()
        : Node("filter_node") {
        alpha_ = 0.1;         // 低通系数：越小越平滑，但延迟越大
        window_size_ = 11;    // 中值窗口大小（奇数）

        sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "signal_raw", 10, std::bind(&FilterNode::callback, this, std::placeholders::_1));

        pub_lpf_ = this->create_publisher<std_msgs::msg::Float64>("signal_lpf", 10);
        pub_median_ = this->create_publisher<std_msgs::msg::Float64>("signal_median", 10);
    }

private:
    void callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double x = msg->data; // 刚收到的原始值

        // ===== 低通滤波器（一阶 IIR）=====
        // 公式：y = α·x + (1-α)·y_prev
        // 第一次直接拿 x 当初值，避免从 0 慢慢爬
        if (!lpf_initialized_) {
            lpf_prev_ = x;
            lpf_initialized_ = true;
        }
        double lpf_out = alpha_ * x + (1.0 - alpha_) * lpf_prev_;
        lpf_prev_ = lpf_out;

        // ===== 中值滤波器（滑动窗口）=====
        // 新样本入队，超窗口长度就踢最老的
        window_.push_back(x);
        if ((int)window_.size() > window_size_) {
            window_.pop_front();
        }
        double median_out = median(window_);

        // ===== 把两路结果发出去，给 Foxglove 看 =====
        auto lpf_msg = std_msgs::msg::Float64();
        lpf_msg.data = lpf_out;
        pub_lpf_->publish(lpf_msg);

        auto med_msg = std_msgs::msg::Float64();
        med_msg.data = median_out;
        pub_median_->publish(med_msg);
    }

    // 取中位数：排序后取中间那个
    double median(const std::deque<double>& window) {
        std::vector<double> v(window.begin(), window.end());
        std::sort(v.begin(), v.end());
        int n = v.size();
        if (n % 2 == 1) return v[n/2];
    return (v[n/2 - 1] + v[n/2]) / 2.0;
    }

    double alpha_;
    int window_size_;
    bool lpf_initialized_ = false;
    double lpf_prev_ = 0.0;
    std::deque<double> window_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_lpf_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_median_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
