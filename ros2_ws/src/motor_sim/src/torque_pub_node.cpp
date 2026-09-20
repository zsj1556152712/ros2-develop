#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class TorquePubNode : public rclcpp::Node {
public:
    TorquePubNode()
        : Node("torque_pub_node") {
        // ===== 可配置参数 =====
        // 信号类型: "step" / "square" / "sine"
        declare_parameter("signal_type", "step");
        // 信号幅值（力矩大小，N·m）
        declare_parameter("amplitude", 2.0);
        // 方波/正弦的频率 Hz
        declare_parameter("freq", 0.5);
        // 发布频率 Hz（比电机积分频率低，模拟控制频率）
        declare_parameter("pub_rate", 100.0);
        // 阶跃信号延迟时间（秒），过了这段时间才开始给力矩
        declare_parameter("step_delay", 1.0);

        signal_type_ = get_parameter("signal_type").as_string();
        amplitude_ = get_parameter("amplitude").as_double();
        freq_ = get_parameter("freq").as_double();
        double rate_hz = get_parameter("pub_rate").as_double();
        step_delay_ = get_parameter("step_delay").as_double();
        dt_ = 1.0 / rate_hz;

        RCLCPP_INFO(
            get_logger(), "测试信号源启动: 类型=%s, 幅值=%.2f, 频率=%.1f Hz", signal_type_.c_str(),
            amplitude_, freq_);

        torque_pub_ = this->create_publisher<std_msgs::msg::Float64>("/torque_cmd", 10);

        start_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&TorquePubNode::timerCallback, this));
    }

private:
    void timerCallback() {
        double t = (this->now() - start_time_).seconds();
        double torque = 0.0;

        if (signal_type_ == "step") {
            // 阶跃：过了 step_delay 后突然给恒定力矩
            if (t > step_delay_) {
                torque = amplitude_;
            }
        } else if (signal_type_ == "square") {
            // 方波：正负交替，周期 = 1/freq
            // 用 sin 的正负号来判断
            double s = std::sin(2.0 * M_PI * freq_ * t);
            torque = (s >= 0.0) ? amplitude_ : -amplitude_;
        } else if (signal_type_ == "sine") {
            // 正弦力矩
            torque = amplitude_ * std::sin(2.0 * M_PI * freq_ * t);
        }

        auto msg = std_msgs::msg::Float64();
        msg.data = torque;
        torque_pub_->publish(msg);

        // 每 1 秒打一次日志，不刷屏
        if ((int)(t * 10) % 10 == 0 && t > 0.0) {
            RCLCPP_INFO(get_logger(), "t=%.2f, 力矩=%.3f N·m", t, torque);
        }
    }

    // 参数
    std::string signal_type_;
    double amplitude_;
    double freq_;
    double step_delay_;
    double dt_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorquePubNode>());
    rclcpp::shutdown();
    return 0;
}
