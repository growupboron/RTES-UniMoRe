// cart_pole_controller_tw.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>
#include <vector>
#include <limits>

using namespace std::chrono_literals;

class CartPoleControllerTw : public rclcpp::Node {
public:
    CartPoleControllerTw() : Node("cart_pole_controller_tw"), prev_error_(0.0), integral_(0.0), tuning_step_(0) {
        this->declare_parameter<double>("kp", 10.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);

        force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/cart/force", 10);
        state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cart/pole_state", 10, std::bind(&CartPoleControllerTw::state_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CartPoleControllerTw::control_loop, this));
        last_time_ = this->now();

        reset_simulation();
        zero_force();
        init_autotune();
    }

private:
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        angle_ = msg->position[0];
        RCLCPP_INFO(this->get_logger(), "Angle: %f", angle_);
    }

    void control_loop() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);

        double error = 0.0 - angle_;  // Setpoint is 0.0 (vertical position)

        double derivative = (error - prev_error_) / dt;
        integral_ += error * dt;
        double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;

        auto force_msg = geometry_msgs::msg::Wrench();
        force_msg.force.y = control_signal;
        force_publisher_->publish(force_msg);

        RCLCPP_INFO(this->get_logger(), "Control Signal: %f", control_signal);

        prev_error_ = error;
    }

    void zero_force() {
        auto zero_force_msg = geometry_msgs::msg::Wrench();
        zero_force_msg.force.y = 0.0;
        force_publisher_->publish(zero_force_msg);
    }

    void reset_simulation() {
        auto client = this->create_client<std_srvs::srv::Empty>("/reset_simulation");
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = client->async_send_request(request);
    }

    void init_autotune() {
        RCLCPP_INFO(this->get_logger(), "Initializing auto-tuning algorithm...");

        p_ = {this->get_parameter("kp").as_double(), this->get_parameter("ki").as_double(), this->get_parameter("kd").as_double()};
        dp_ = {1.0, 1.0, 1.0};  // Initial adjustments
        best_error_ = std::numeric_limits<double>::max();
        tuning_step_ = 0;

        tune_pid();
    }

    void tune_pid() {
        while (sum(dp_) > 0.001) {  // Threshold for tuning
            for (int i = 0; i < p_.size(); ++i) {
                p_[i] += dp_[i];
                set_pid_parameters(p_);
                double error = measure_error();

                if (error < best_error_) {
                    best_error_ = error;
                    dp_[i] *= 1.1;
                } else {
                    p_[i] -= 2 * dp_[i];
                    set_pid_parameters(p_);
                    error = measure_error();

                    if (error < best_error_) {
                        best_error_ = error;
                        dp_[i] *= 1.1;
                    } else {
                        p_[i] += dp_[i];
                        dp_[i] *= 0.9;
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "Tuning step %d: kp=%f, ki=%f, kd=%f", tuning_step_++, p_[0], p_[1], p_[2]);
        }

        RCLCPP_INFO(this->get_logger(), "Auto-tuning complete: kp=%f, ki=%f, kd=%f", p_[0], p_[1], p_[2]);
        set_pid_parameters(p_);
    }

    double measure_error() {
        double total_error = 0.0;
        const double test_duration = 5.0; // seconds
        auto start_time = this->now();

        while ((this->now() - start_time).seconds() < test_duration) {
            double error = 0.0 - angle_; // Setpoint is 0.0 (vertical position)
            total_error += std::abs(error);
            rclcpp::sleep_for(50ms); // Assuming a control loop of 50ms
        }

        return total_error;
    }

    void set_pid_parameters(const std::vector<double>& params) {
        this->set_parameter(rclcpp::Parameter("kp", params[0]));
        this->set_parameter(rclcpp::Parameter("ki", params[1]));
        this->set_parameter(rclcpp::Parameter("kd", params[2]));
    }

    double sum(const std::vector<double>& v) {
        double total = 0.0;
        for (double val : v) {
            total += val;
        }
        return total;
    }

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    double kp_, ki_, kd_;
    double prev_error_, integral_;
    rclcpp::Time last_time_;
    double angle_;
    std::vector<double> p_, dp_;
    double best_error_;
    int tuning_step_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartPoleControllerTw>());
    rclcpp::shutdown();
    return 0;
}