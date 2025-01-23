#ifndef DIRECT_KIN_SERVER_HPP
#define DIRECT_KIN_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <custom_msg_interfaces/srv/compute_dir_kin.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <vector>

class DirectKinServer : public rclcpp::Node
{
public:
    DirectKinServer();

private:
    // Service callback function
    void computeDirectKinematics(
        const std::shared_ptr<custom_msg_interfaces::srv::ComputeDirKin::Request> request,
        std::shared_ptr<custom_msg_interfaces::srv::ComputeDirKin::Response> response);

    // Utility functions
    Eigen::Matrix4d Tij(double th, double alpha, double d, double a);
    void ur5Direct(const std::vector<double>& Th, double scaleFactor, Eigen::Vector3d& pe, Eigen::Matrix3d& Re, std::vector<Eigen::Matrix4d>& Tm);

    rclcpp::Service<custom_msg_interfaces::srv::ComputeDirKin>::SharedPtr service_;
};

#endif // DIRECT_KIN_SERVER_HPP
