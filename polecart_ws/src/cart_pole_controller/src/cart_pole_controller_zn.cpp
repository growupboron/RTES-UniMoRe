#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class CartPoleControllerZN : public rclcpp::Node {
public:
    CartPoleControllerZN() : Node("cart_pole_controller_zn"), prev_error_(0.0), integral_(0.0), tuning_step_(0) {
        this->declare_parameter<double>("kp", 1.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);

        force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/cart/force", 10);
        state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cart/pole_state", 10, std::bind(&CartPoleControllerZN::state_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CartPoleControllerZN::control_loop, this));
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
        RCLCPP_INFO(this->get_logger(), "Initializing Ziegler-Nichols auto-tuning...");

        double Ku = 0.0;
        double Tu = 0.0;

        // Step 1: Set Ki and Kd to zero, and increase Kp until the system oscillates
        this->set_parameter(rclcpp::Parameter("ki", 0.0));
        this->set_parameter(rclcpp::Parameter("kd", 0.0));
        bool oscillating = false;
        double kp_test = 1.0;
        while (!oscillating) {
            this->set_parameter(rclcpp::Parameter("kp", kp_test));
            if (check_oscillation()) {
                Ku = kp_test;
                Tu = measure_oscillation_period();
                oscillating = true;
            } else {
                kp_test += 1.0;
            }
        }

        // Step 2: Calculate PID parameters using Ziegler-Nichols method
        double kp = 0.6 * Ku;
        double ki = 2.0 * kp / Tu;
        double kd = kp * Tu / 8.0;

        this->set_parameter(rclcpp::Parameter("kp", kp));
        this->set_parameter(rclcpp::Parameter("ki", ki));
        this->set_parameter(rclcpp::Parameter("kd", kd));

        RCLCPP_INFO(this->get_logger(), "Auto-tuning complete: Ku=%f, Tu=%f, kp=%f, ki=%f, kd=%f", Ku, Tu, kp, ki, kd);
    }

    bool check_oscillation() {
        double total_error = 0.0;
        const double test_duration = 5.0; // seconds
        auto start_time = this->now();

        while ((this->now() - start_time).seconds() < test_duration) {
            double error = 0.0 - angle_; // Setpoint is 0.0 (vertical position)
            total_error += std::abs(error);
            rclcpp::sleep_for(50ms); // Assuming a control loop of 50ms
        }

        return (total_error > 0.5 && total_error < 1.5);
    }

    double measure_oscillation_period() {
        std::vector<double> error_values;
        const double measure_duration = 5.0; // seconds
        auto start_time = this->now();

        while ((this->now() - start_time).seconds() < measure_duration) {
            double error = 0.0 - angle_; // Setpoint is 0.0 (vertical position)
            error_values.push_back(error);
            rclcpp::sleep_for(50ms); // Assuming a control loop of 50ms
        }

        int zero_crossings = 0;
        for (size_t i = 1; i < error_values.size(); ++i) {
            if ((error_values[i] > 0 && error_values[i-1] < 0) || (error_values[i] < 0 && error_values[i-1] > 0)) {
                zero_crossings++;
            }
        }

        return measure_duration / zero_crossings;
    }

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    double kp_, ki_, kd_;
    double prev_error_, integral_;
    rclcpp::Time last_time_;
    double angle_;
    int tuning_step_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartPoleControllerZN>());
    rclcpp::shutdown();
    return 0;
}