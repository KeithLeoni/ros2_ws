#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <custom_msg_interfaces/srv/compute_path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>
#include <rclcpp/qos.hpp>  // Include QoS headers


class SlerpPathPlanning : public rclcpp::Node {
public:
    SlerpPathPlanning() : Node("slerp_path_planning") {
        service_ = this->create_service<custom_msg_interfaces::srv::ComputePath>(
            "compute_path",
            std::bind(&SlerpPathPlanning::compute_path_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "linear_trajectory",
            rclcpp::QoS(10).transient_local().reliable()
        );

        RCLCPP_INFO(this->get_logger(), "SLERP Path Planning Service Ready");
    }

private:
    rclcpp::Service<custom_msg_interfaces::srv::ComputePath>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;

    void compute_path_callback(
        const std::shared_ptr<custom_msg_interfaces::srv::ComputePath::Request> request,
        std::shared_ptr<custom_msg_interfaces::srv::ComputePath::Response> response) 
    {
        geometry_msgs::msg::Pose start_pose = request->pose_start;
        geometry_msgs::msg::Pose end_pose = request->pose_end;
        int num_interpolations = request->num_interpolations;

        // Perform interpolation
        std::vector<geometry_msgs::msg::Pose> interpolated_poses = interpolate_linear(start_pose, end_pose, num_interpolations);
        response->poses = interpolated_poses;

        // Publish for visualization in RViz
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->now();
        pose_array.header.frame_id = "base";  // Modify as needed
        pose_array.poses = interpolated_poses;
        publisher_->publish(pose_array);
    }

    std::vector<geometry_msgs::msg::Pose> interpolate_linear(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& end_pose,
        int num_interpolations)
    {
        std::vector<geometry_msgs::msg::Pose> poses;
        
        tf2::Quaternion q_start(
            start_pose.orientation.x, start_pose.orientation.y,
            start_pose.orientation.z, start_pose.orientation.w
        );
        tf2::Quaternion q_end(
            end_pose.orientation.x, end_pose.orientation.y,
            end_pose.orientation.z, end_pose.orientation.w
        );

        for (int i = 0; i <= num_interpolations; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(num_interpolations);

            geometry_msgs::msg::Pose interp_pose;

            // Linear interpolation for position
            interp_pose.position.x = (1 - t) * start_pose.position.x + t * end_pose.position.x;
            interp_pose.position.y = (1 - t) * start_pose.position.y + t * end_pose.position.y;
            interp_pose.position.z = (1 - t) * start_pose.position.z + t * end_pose.position.z;

            // SLERP for orientation
            tf2::Quaternion q_interp = q_start.slerp(q_end, t);
            interp_pose.orientation.x = q_interp.x();
            interp_pose.orientation.y = q_interp.y();
            interp_pose.orientation.z = q_interp.z();
            interp_pose.orientation.w = q_interp.w();

            poses.push_back(interp_pose);
        }
        return poses;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlerpPathPlanning>());
    rclcpp::shutdown();
    return 0;
}

