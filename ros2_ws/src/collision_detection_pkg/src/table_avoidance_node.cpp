#include <memory>
#include <rclcpp/rclcpp.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Standard messages
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class EndEffectorHeightChecker : public rclcpp::Node
{
public:
  EndEffectorHeightChecker()
  : Node("end_effector_height_checker"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Create a publisher to publish shutdown signals
    shutdown_pub_ = this->create_publisher<std_msgs::msg::Bool>("shutdown", 10);

    // Create a timer to periodically check TF and publish status
    timer_ = this->create_wall_timer(
      500ms,  // e.g., check at 2 Hz
      std::bind(&EndEffectorHeightChecker::timerCallback, this)
    );
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Try to look up the transform from "base_link" to "tool0"
    try
    {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link",  // source frame
        "tool0",      // target frame
        tf2::TimePointZero
      );
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }

    // Extract the z-coordinate of the end effector
    double end_effector_z = transform_stamped.transform.translation.z;
    double z_threshold = 0.87 + 0.10; // For example

    if (end_effector_z < z_threshold)
    {
      RCLCPP_WARN(this->get_logger(), "End effector is too low! Z=%.3f", end_effector_z);
      std_msgs::msg::Bool shutdown_msg;
      shutdown_msg.data = true;
      shutdown_pub_->publish(shutdown_msg);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "End effector height OK. Z=%.3f", end_effector_z);
    }
  }

  // TF objects
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EndEffectorHeightChecker>());
  rclcpp::shutdown();
  return 0;
}

