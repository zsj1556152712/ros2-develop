#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class MotorSimNode : public rclcpp::Node {
public:
    MotorSimNode()
        : Node("motor_sim_node") {
        // ===== 可配置参数 =====
        // 转动惯量 J (kg·m²)，越大越难加速
        declare_parameter("J", 1.0);
        // 阻尼系数 B，越大越容易停下来
        declare_parameter("B", 0.5);
        // 状态更新频率 Hz，越高积分越准
        declare_parameter("update_rate", 1000.0);

        J_ = get_parameter("J").as_double();
        B_ = get_parameter("B").as_double();
        double rate_hz = get_parameter("update_rate").as_double();
        dt_ = 1.0 / rate_hz; // 步长，秒

        RCLCPP_INFO(
            get_logger(), "电机模拟器启动: J=%.2f, B=%.2f, 更新频率=%.0f Hz", J_, B_, rate_hz);

        // ===== 订阅力矩命令 =====
        torque_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/torque_cmd", 10,
            std::bind(&MotorSimNode::torqueCallback, this, std::placeholders::_1));

        // ===== 发布状态 =====
        omega_pub_ = this->create_publisher<std_msgs::msg::Float64>("/motor/omega", 10);
        theta_pub_ = this->create_publisher<std_msgs::msg::Float64>("/motor/theta", 10);

        // ===== 定时器：高频积分 =====
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&MotorSimNode::updateCallback, this));
    }

private:
    // 收到力矩命令，存下来（下次积分用）
    void torqueCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_torque_ = msg->data;
    }

    // 高频积分更新
    void updateCallback() {
        // 半隐式欧拉：先更新角速度，再用新角速度更新角度
        // 公式：omega_new = omega + (T - B*omega) / J * dt
        omega_ += (current_torque_ - B_ * omega_) / J_ * dt_;
        theta_ += omega_ * dt_;

        // 发布状态
        auto omega_msg = std_msgs::msg::Float64();
        omega_msg.data = omega_;
        omega_pub_->publish(omega_msg);

        auto theta_msg = std_msgs::msg::Float64();
        theta_msg.data = theta_;
        theta_pub_->publish(theta_msg);
    }

    // 参数
    double J_;
    double B_;
    double dt_;

    // 状态
    double theta_ = 0.0; // 当前角度（弧度），从零开始
    double omega_ = 0.0; // 当前角速度（弧度/秒）

    // 当前力矩命令
    double current_torque_ = 0.0;

    // ROS2 对象
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr omega_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorSimNode>());
    rclcpp::shutdown();
    return 0;
}
