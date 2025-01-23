#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Dense>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "custom_msg_interfaces/srv/compute_ik.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Forward-declare the class
namespace motion_pkg
{
class InverseKinServer : public rclcpp::Node
{
public:
  explicit InverseKinServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
private:
  // Callback for the "ComputeIK" service
  void computeIKCallback(
    const std::shared_ptr<custom_msg_interfaces::srv::ComputeIK::Request> request,
    std::shared_ptr<custom_msg_interfaces::srv::ComputeIK::Response>      response);

  // Our UR5 IK solver function adapted from your code
  // (uses single-precision floats internally, but we will convert to double in the response)
  Eigen::MatrixXd ur5Inverse(
    const Eigen::Vector3f & p60, 
    const Eigen::Matrix3f & R60, 
    float scaleFactor
  );

  // A small helper function to check for near-zero float values
  inline bool almzero(float x) const
  {
    return std::fabs(x) < 1e-7;
  }

private:
  rclcpp::Service<custom_msg_interfaces::srv::ComputeIK>::SharedPtr service_;
};
}  // namespace motion_pkg
