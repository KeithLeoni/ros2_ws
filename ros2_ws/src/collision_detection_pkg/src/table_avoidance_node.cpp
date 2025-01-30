#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

// This node checks that the height of the ee is above a threshold to avoid collision
class EndEffectorHeightChecker : public rclcpp::Node {
public:
    EndEffectorHeightChecker() : Node("end_effector_height_checker") {
        // Subscription to the "end_effector_position" topic, when ee position changes the function topic_callback checks the height
        subscription_ = this->create_subscription<std_msgs::msg::Float64>("end_effector_position", 10,
            std::bind(&EndEffectorHeightChecker::topic_callback, this, std::placeholders::_1));
        // It also publishes on a "shutdown" topic to signal the other node to shutdown when the ee is too low
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("shutdown", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Extract the z-coordinate from the message
        double end_effector_z = msg->data;

        // Define the threshold for the end effector's z-coordinate
        double z_threshold = 0.87 + 0.10;

        if (end_effector_z < z_threshold) {
            RCLCPP_WARN(this->get_logger(), "End effector is too low!");
            std_msgs::msg::Bool shutdown_msg;
            shutdown_msg.data = true;
            publisher_->publish(shutdown_msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "End effector height is within acceptable range.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndEffectorHeightChecker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;  // Optional, but good practice
}

