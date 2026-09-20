#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

// PID控制器类
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double i_limit, double out_limit)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , i_limit_(i_limit)
        , out_limit_(out_limit)
        , integral_(0.0)
        , prev_error_(0.0)
        , initialized_(false) {}

    // 计算PID输出
    double calculate(double target, double measured, double dt) {
        double error = target - measured;

        // 积分项（带限幅，防止积分饱和）
        integral_ += error * dt;
        if (integral_ > i_limit_)
            integral_ = i_limit_;
        if (integral_ < -i_limit_)
            integral_ = -i_limit_;

        // 微分项
        double derivative = 0.0;
        if (initialized_ && dt > 0.0) {
            derivative = (error - prev_error_) / dt;
        }
        prev_error_ = error;
        initialized_ = true;

        // PID总和
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 输出限幅
        if (output > out_limit_)
            output = out_limit_;
        if (output < -out_limit_)
            output = -out_limit_;

        return output;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        initialized_ = false;
    }

private:
    double kp_, ki_, kd_;
    double i_limit_;
    double out_limit_;
    double integral_;
    double prev_error_;
    bool initialized_;
};

// PID节点
class PIDNode : public rclcpp::Node {
public:
    PIDNode()
        : Node("pid_node") {
        // 声明参数（可以用 ros-args 改）
        this->declare_parameter<double>("kp", 1.5);
        this->declare_parameter<double>("ki", 0.3);
        this->declare_parameter<double>("kd", 0.3);
        this->declare_parameter<double>("target_speed", 4.0); // 目标角速度 rad/s
        this->declare_parameter<double>("i_limit", 10.0);     // 积分限幅
        this->declare_parameter<double>("out_limit", 10.0);   // 力矩输出限幅
        this->declare_parameter<int>("control_rate", 1000);   // 控制频率 Hz

        // 读取参数
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        double target_speed = this->get_parameter("target_speed").as_double();
        double i_limit = this->get_parameter("i_limit").as_double();
        double out_limit = this->get_parameter("out_limit").as_double();
        int control_rate = this->get_parameter("control_rate").as_int();

        // 创建PID控制器
        pid_ = std::make_unique<PIDController>(kp, ki, kd, i_limit, out_limit);
        target_speed_ = target_speed;

        // 订阅实际速度
        omega_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/omega", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { current_omega_ = msg->data; });

        // 发布控制力矩
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64>("/torque_cmd", 10);

        // 控制定时器（1000Hz）
        double dt = 1.0 / control_rate;
        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), [this, dt]() {
            // PID计算
            double torque = pid_->calculate(target_speed_, current_omega_, dt);

            // 发布力矩
            auto msg = std_msgs::msg::Float64();
            msg.data = torque;
            torque_pub_->publish(msg);
        });

        RCLCPP_INFO(
            this->get_logger(), "PID节点启动：目标速度=%.2f rad/s, Kp=%.2f, Ki=%.2f, Kd=%.2f",
            target_speed_, kp, ki, kd);
    }

private:
    std::unique_ptr<PIDController> pid_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr omega_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_omega_ = 0.0;
    double target_speed_ = 4.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDNode>());
    rclcpp::shutdown();
    return 0;
}
