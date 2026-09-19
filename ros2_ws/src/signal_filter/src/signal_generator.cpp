#include <cmath>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class SignalGenerator : public rclcpp::Node {
public:
    SignalGenerator()
        : Node("signal_generator") {
        amplitude_ = 0.3;           // 幅值 1.0（你可以改）
        freq_ = 20.0;               // 正弦频率 20Hz
        sigma_ = amplitude_ * 0.01; // 噪声标准差 = 幅值的 1% = 0.01

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("signal_raw", 10);

        // 500Hz 定时器：每 2ms 触发一次回调
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), std::bind(&SignalGenerator::timer_callback, this));

        // 高斯噪声生成器：均值 0，标准差 sigma_
        noise_gen_ = std::normal_distribution<double>(0.0, sigma_);

        start_time_ = this->now();
    }

private:
    void timer_callback() {
        double t = (this->now() - start_time_).seconds();             // 从启动到现在的秒数
        double clean = amplitude_ * std::sin(2.0 * M_PI * freq_ * t); // 干净的正弦
        double noise = noise_gen_(rng_);                              // 随机噪声
        double value = clean + noise;                                 // 叠加噪声

        auto msg = std_msgs::msg::Float64();
        msg.data = value;
        publisher_->publish(msg);

        // 每 100 次打一次日志（500Hz 全打会刷屏）
        if (count_++ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "t=%.3f, raw=%.4f", t, value);
        }
    }

    double amplitude_;
    double freq_;
    double sigma_;
    std::default_random_engine rng_{42}; // 随机数引擎，固定种子方便复现
    std::normal_distribution<double> noise_gen_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    int count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}
