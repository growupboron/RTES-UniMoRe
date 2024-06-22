#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CartPoleController : public rclcpp::Node {
public:
    CartPoleController() : Node("cart_pole_controller"), kp_(100.0), ki_(70.0), kd_(50.0), prev_error_(0.0), integral_(0.0) {
        force_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/cart/force", 10);
        state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cart/pole_state", 10, std::bind(&CartPoleController::state_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CartPoleController::control_loop, this));
        last_time_ = this->now();

        reset_simulation();
        zero_force();
    }

private:
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        angle_ = msg->position[0];
    }

    void control_loop() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double error = 0.0 - angle_;  // Setpoint is 0.0 (vertical position)

        double derivative = (error - prev_error_) / dt;
        integral_ += error * dt;
        double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;

        auto force_msg = geometry_msgs::msg::Wrench();
        force_msg.force.y = control_signal;
        force_publisher_->publish(force_msg);

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

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    double kp_, ki_, kd_;
    double prev_error_, integral_;
    rclcpp::Time last_time_;
    double angle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartPoleController>());
    rclcpp::shutdown();
    return 0;
}