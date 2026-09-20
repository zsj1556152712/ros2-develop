#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

// 角度差（走最短路径，劣弧）
double angle_diff(double target, double current) {
    double diff = target - current;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double i_limit, double out_limit)
        : kp_(kp), ki_(ki), kd_(kd), i_limit_(i_limit), out_limit_(out_limit),
          integral_(0.0), prev_error_(0.0), initialized_(false) {}

    double calculate(double error, double dt) {
        integral_ += error * dt;
        if (integral_ > i_limit_) integral_ = i_limit_;
        if (integral_ < -i_limit_) integral_ = -i_limit_;

        double derivative = 0.0;
        if (initialized_ && dt > 0.0) {
            derivative = (error - prev_error_) / dt;
        }
        prev_error_ = error;
        initialized_ = true;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        if (output > out_limit_) output = out_limit_;
        if (output < -out_limit_) output = -out_limit_;

        return output;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        initialized_ = false;
    }

private:
    double kp_, ki_, kd_;
    double i_limit_, out_limit_;
    double integral_, prev_error_;
    bool initialized_;
};

class AnglePIDNode : public rclcpp::Node {
public:
    AnglePIDNode() : Node("angle_pid_node") {
        this->declare_parameter<double>("pos_kp", 0.1);
        this->declare_parameter<double>("pos_ki", 0.0);
        this->declare_parameter<double>("pos_kd", 0.05);
        this->declare_parameter<double>("pos_i_limit", 5.0);
        this->declare_parameter<double>("pos_out_limit", 1.0);

        this->declare_parameter<double>("vel_kp", 1.5);
        this->declare_parameter<double>("vel_ki", 0.3);
        this->declare_parameter<double>("vel_kd", 0.3);
        this->declare_parameter<double>("vel_i_limit", 10.0);
        this->declare_parameter<double>("vel_out_limit", 10.0);

        this->declare_parameter<double>("target_angle", 5 * M_PI / 2);

        double pos_kp = this->get_parameter("pos_kp").as_double();
        double pos_ki = this->get_parameter("pos_ki").as_double();
        double pos_kd = this->get_parameter("pos_kd").as_double();
        double pos_i_limit = this->get_parameter("pos_i_limit").as_double();
        double pos_out_limit = this->get_parameter("pos_out_limit").as_double();

        double vel_kp = this->get_parameter("vel_kp").as_double();
        double vel_ki = this->get_parameter("vel_ki").as_double();
        double vel_kd = this->get_parameter("vel_kd").as_double();
        double vel_i_limit = this->get_parameter("vel_i_limit").as_double();
        double vel_out_limit = this->get_parameter("vel_out_limit").as_double();

        target_angle_ = this->get_parameter("target_angle").as_double();

        pos_pid_ = std::make_unique<PIDController>(pos_kp, pos_ki, pos_kd, pos_i_limit, pos_out_limit);
        vel_pid_ = std::make_unique<PIDController>(vel_kp, vel_ki, vel_kd, vel_i_limit, vel_out_limit);

        theta_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/theta", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                current_theta_ = msg->data;
            });

        omega_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/omega", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                current_omega_ = msg->data;
            });

        torque_pub_ = this->create_publisher<std_msgs::msg::Float64>("/torque_cmd", 10);

        // 外环（位置环）：100Hz
        double pos_dt = 1.0 / 100.0;
        pos_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(pos_dt),
            [this, pos_dt]() {
                double pos_error = angle_diff(target_angle_, current_theta_);
                target_omega_ = pos_pid_->calculate(pos_error, pos_dt);
            });

        // 内环（速度环）：1000Hz
        double vel_dt = 1.0 / 1000.0;
        vel_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(vel_dt),
            [this, vel_dt]() {
                double vel_error = target_omega_ - current_omega_;
                double torque = vel_pid_->calculate(vel_error, vel_dt);

                auto msg = std_msgs::msg::Float64();
                msg.data = torque;
                torque_pub_->publish(msg);
            });

        RCLCPP_INFO(this->get_logger(), "双环PID节点启动：目标角度=%.2f rad (%.1f°)",
                    target_angle_, target_angle_ * 180.0 / M_PI);
    }

private:
    std::unique_ptr<PIDController> pos_pid_;
    std::unique_ptr<PIDController> vel_pid_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr omega_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_pub_;
    rclcpp::TimerBase::SharedPtr pos_timer_;
    rclcpp::TimerBase::SharedPtr vel_timer_;
    double current_theta_ = 0.0;
    double current_omega_ = 0.0;
    double target_omega_ = 0.0;
    double target_angle_ = M_PI / 2;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnglePIDNode>());
    rclcpp::shutdown();
    return 0;
}
