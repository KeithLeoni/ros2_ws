#include "motion_pkg/direct_kin_server.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <vector>

DirectKinServer::DirectKinServer() : Node("direct_kin_server")
{
    // Create service
    service_ = this->create_service<custom_msg_interfaces::srv::ComputeDirKin>(
        "compute_dir_kin",
        std::bind(
            &DirectKinServer::computeDirectKinematics, this,
            std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Direct Kinematics Service is ready.");
}

void DirectKinServer::computeDirectKinematics(
    const std::shared_ptr<custom_msg_interfaces::srv::ComputeDirKin::Request> request,
    std::shared_ptr<custom_msg_interfaces::srv::ComputeDirKin::Response> response)
{
    try
    {
        Eigen::Vector3d end_effector_position;
        Eigen::Matrix3d end_effector_orientation;
        std::vector<Eigen::Matrix4d> Tm;

        // Compute direct kinematics
        ur5Direct(request->joints, 1.0, end_effector_position, end_effector_orientation, Tm);

        // Populate response
        response->final_pose.position.x = end_effector_position(0);
        response->final_pose.position.y = end_effector_position(1);
        response->final_pose.position.z = end_effector_position(2);

        Eigen::Quaterniond quaternion(end_effector_orientation);
        response->final_pose.orientation.x = quaternion.x();
        response->final_pose.orientation.y = quaternion.y();
        response->final_pose.orientation.z = quaternion.z();
        response->final_pose.orientation.w = quaternion.w();

        response->status_message = "Direct kinematics calculated successfully";
        response->frame_id = request->frame_id;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in direct kinematics calculation: %s", e.what());
        response->status_message = "Failed to calculate direct kinematics";
    }
}

Eigen::Matrix4d DirectKinServer::Tij(double th, double alpha, double d, double a)
{
    Eigen::Matrix4d T;
    T << std::cos(th), -std::sin(th) * std::cos(alpha), std::sin(th) * std::sin(alpha), a * std::cos(th),
         std::sin(th), std::cos(th) * std::cos(alpha), -std::cos(th) * std::sin(alpha), a * std::sin(th),
         0, std::sin(alpha), std::cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

void DirectKinServer::ur5Direct(
    const std::vector<double>& Th, double scaleFactor, Eigen::Vector3d& pe,
    Eigen::Matrix3d& Re, std::vector<Eigen::Matrix4d>& Tm)
{
    std::vector<double> A = {0, -0.425, -0.3922, 0, 0, 0};
    for (auto& a : A) a *= scaleFactor;

    std::vector<double> D = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
    for (auto& d : D) d *= scaleFactor;

    std::vector<double> Alpha = {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};

    Tm.push_back(Tij(Th[0], Alpha[0], D[0], A[0]));
    Tm.push_back(Tij(Th[1], Alpha[1], D[1], A[1]));
    Tm.push_back(Tij(Th[2], Alpha[2], D[2], A[2]));
    Tm.push_back(Tij(Th[3], Alpha[3], D[3], A[3]));
    Tm.push_back(Tij(Th[4], Alpha[4], D[4], A[4]));
    Tm.push_back(Tij(Th[5], Alpha[5], D[5], A[5]));

    Eigen::Matrix4d T60 = Tm[0] * Tm[1] * Tm[2] * Tm[3] * Tm[4] * Tm[5];
    pe = T60.block<3, 1>(0, 3);
    Re = T60.block<3, 3>(0, 0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectKinServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
