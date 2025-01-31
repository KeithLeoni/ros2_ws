#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <custom_msg_interfaces/srv/compute_trajectory.hpp>
#include <custom_msg_interfaces/srv/compute_ik.hpp>


#include <vector>
#include <array>
#include <sstream>
#include <limits>
#include <algorithm>
#include <chrono>
#include <cmath>  // For std::isnan
#include <iostream>
#include <Eigen/Dense>
using namespace std::chrono_literals;  

// ---------------------------------------------------------------------------
// 1) Global WAYPOINTS (Populated in the node after IK calls)
// ---------------------------------------------------------------------------
std::vector<std::array<double, 6>> waypoints;

// ---------------------------------------------------------------------------
// 2) Joint Names for Our 6-DOF Arm
// ---------------------------------------------------------------------------
const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

// ---------------------------------------------------------------------------
// 3) Compute Cubic Spline Coefficients
//    For a segment from q0->q1 in time T, given velocities v0->v1
//    s(t) = a0 + a1*t + a2*t^2 + a3*t^3
//    subject to position & velocity constraints at t=0 and t=T
// ---------------------------------------------------------------------------
std::array<double, 4> compute_cubic_coefficients(double q0, double q1,
                                                 double v0, double v1,
                                                 double T)
{
    double a0 = q0;
    double a1 = v0;
    double a2 = (3 * (q1 - q0) / (T * T)) - (2 * v0 / T) - (v1 / T);
    double a3 = (-2 * (q1 - q0) / (T * T * T)) + ((v1 + v0) / (T * T));
    return {a0, a1, a2, a3};
}

// ---------------------------------------------------------------------------
// 4) Generate a Smooth Trajectory Using Cubic Interpolation
//    Takes the global waypoints (N+1 or however many) and a segment_time
//    for each segment, returns a JointTrajectory with interpolation
// ---------------------------------------------------------------------------
trajectory_msgs::msg::JointTrajectory generate_cubic_trajectory(
    const std::vector<std::array<double, 6>>& waypoints,
    double segment_time)
{
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = JOINT_NAMES;
    double total_time = 0.0;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        std::array<double, 6> q0 = waypoints[i];
        std::array<double, 6> q1 = waypoints[i + 1];

        std::array<double, 6> v0, v1;

        // First waypoint: velocity = 0
        if (i == 0) {
            v0.fill(0.0);
            for (size_t j = 0; j < 6; ++j) {
                v1[j] = (waypoints[i + 2][j] - q0[j]) / (2.0 * segment_time);
            }
        }
        // Last waypoint: velocity = 0
        else if (i == waypoints.size() - 2) {
            v1.fill(0.0);
            for (size_t j = 0; j < 6; ++j) {
                v0[j] = (q1[j] - waypoints[i - 1][j]) / (2.0 * segment_time);
            }
        }
        // Intermediate waypoints: enforce v0 = v1
        else {
            for (size_t j = 0; j < 6; ++j) {
                double avg_velocity = (waypoints[i + 1][j] - waypoints[i - 1][j]) / (2.0 * segment_time);
                v0[j] = avg_velocity;
                v1[j] = avg_velocity;
            }
        }

        // Compute cubic polynomial coefficients
        std::array<std::array<double, 4>, 6> coefficients;
        for (size_t j = 0; j < 6; ++j) {
            coefficients[j] = compute_cubic_coefficients(q0[j], q1[j], v0[j], v1[j], segment_time);
        }

        // Generate interpolated points
        int steps = 80;
        for (int step = 0; step <= steps; ++step) {
            double t = (step / static_cast<double>(steps)) * segment_time;
            trajectory_msgs::msg::JointTrajectoryPoint interpolated_point;
            interpolated_point.positions.resize(6);
            interpolated_point.velocities.resize(6);

            for (size_t j = 0; j < 6; ++j) {
                double a0 = coefficients[j][0];
                double a1 = coefficients[j][1];
                double a2 = coefficients[j][2];
                double a3 = coefficients[j][3];

                double pos = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
                double vel = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t;

                interpolated_point.positions[j] = pos;
                interpolated_point.velocities[j] = vel;
            }

            double dt = segment_time / static_cast<double>(steps);
            total_time += dt;
            interpolated_point.time_from_start = rclcpp::Duration::from_seconds(total_time);
            traj_msg.points.push_back(interpolated_point);
        }
    }

    return traj_msg;
}



// ---------------------------------------------------------------------------
// 5) The Node Class for "compute_trajectory_service"
// ---------------------------------------------------------------------------
class ComputeTrajectoryService : public rclcpp::Node {
public:
    ComputeTrajectoryService()
    : Node("compute_trajectory_service"), received_initial_joints_(false)
    {
        // Create the service
        service_ = this->create_service<custom_msg_interfaces::srv::ComputeTrajectory>(
            "compute_trajectory",
            std::bind(&ComputeTrajectoryService::compute_trajectory_callback,
                      this, std::placeholders::_1, std::placeholders::_2)
        );

        // Subscribe to /joint_states topic
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ComputeTrajectoryService::joint_state_callback, this, std::placeholders::_1)
        );

        // Create IK client node
        ik_client_node_ = std::make_shared<rclcpp::Node>("compute_ik_client_node");
        ik_client_ = ik_client_node_->create_client<custom_msg_interfaces::srv::ComputeIK>("/compute_ik");

        RCLCPP_INFO(this->get_logger(), "ComputeTrajectoryService node ready on 'compute_trajectory'");
        RCLCPP_INFO(this->get_logger(), "Subscribed to '/joint_states' to capture initial joint values");
    }

private:
    rclcpp::Service<custom_msg_interfaces::srv::ComputeTrajectory>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Node::SharedPtr ik_client_node_;
    rclcpp::Client<custom_msg_interfaces::srv::ComputeIK>::SharedPtr ik_client_;

    std::vector<double> initial_joint_array_;
    bool received_initial_joints_;

    // ---------------------------------------------------------------------------
    // Print the 8x6 matrix (8 solutions) for debugging
    // ---------------------------------------------------------------------------
    void print_joint_angles_matrix(const std::vector<double> &matrix) {
        if (matrix.size() != 8 * 6) {
            RCLCPP_WARN(this->get_logger(), "Unexpected matrix size: %zu (expected 8x6)", matrix.size());
            return;
        }

        std::ostringstream ss;
        for (size_t i = 0; i < 8; ++i) {
            ss << "Row " << (i + 1) << ": ";
            for (size_t j = 0; j < 6; ++j) {
                if (j > 0) ss << ", ";
                ss << matrix[i * 6 + j];
            }
            if (i < 7) ss << " | ";
        }
        RCLCPP_INFO(this->get_logger(), "IK solution matrix:\n%s", ss.str().c_str());
    }

    // ---------------------------------------------------------------------------
    // Callback to store the initial joint values (once)
    // ---------------------------------------------------------------------------
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Resize initial_joint_array_ to match the number of joints
        initial_joint_array_.resize(JOINT_NAMES.size(), 0.0);

        for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), JOINT_NAMES[i]);
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                initial_joint_array_[i] = msg->position[index];
            } else {
                RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in /joint_states",
                            JOINT_NAMES[i].c_str());
            }
        }

        // Print the updated joint states
        std::ostringstream oss;
        for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
            oss << JOINT_NAMES[i] << ": " << initial_joint_array_[i];
            if (i < JOINT_NAMES.size() - 1) oss << " | ";
        }
        //RCLCPP_INFO(this->get_logger(), "Updated joint states:\n%s", oss.str().c_str());
    }



    // ---------------------------------------------------------------------------
    // Select the 6-element row from the 8×6 solutions that is closest
    // to prev_joints in Euclidean sense
    // ---------------------------------------------------------------------------
    std::vector<double> select_closest_one(const std::vector<double> &prev_joints,const std::vector<double> &joint_angles_matrix) {
        if (joint_angles_matrix.size() != 8 * 6 || prev_joints.size() != 6) {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid sizes: prev_joints = %zu, joint_angles_matrix = %zu (expected 6 and 48)",
                        prev_joints.size(), joint_angles_matrix.size());
            return {};
        }

        double min_cost = std::numeric_limits<double>::max();
        std::vector<double> best_solution(6, 0.0);

        for (size_t i = 0; i < 8; ++i) {
            double norm = 0.0;
            double penalty = 0.0;
            bool valid = true;

            for (size_t j = 0; j < 6; ++j) {
                double joint_angle = joint_angles_matrix[i * 6 + j];

                // Check for NaN and skip invalid configurations
                if (std::isnan(joint_angle)) {
                    valid = false;
                    break;
                }

                double diff = joint_angle - prev_joints[j];
                norm += diff * diff;

                // Penalize angles outside [-π, π]
                if (joint_angle < -M_PI || joint_angle > M_PI) {
                    penalty += 100.0;  // Arbitrary penalty value
                }
            }

            // Skip invalid configurations
            if (!valid) {
                RCLCPP_WARN(this->get_logger(),
                            "Configuration %zu contains NaN values and will be skipped.", i);
                continue;
            }

            double cost = norm + penalty;
            if (cost < min_cost) {
                min_cost = cost;
                for (size_t j = 0; j < 6; ++j) {
                    best_solution[j] = joint_angles_matrix[i * 6 + j];
                }
            }
        }

        if (min_cost == std::numeric_limits<double>::max()) {
            RCLCPP_WARN(this->get_logger(), "No valid solution found.");
            return {};
        }

        RCLCPP_INFO(this->get_logger(), "Selected solution with cost: %f", min_cost);
        return best_solution;
    }

    //define the function to check for singularities
    bool ur5_singAvoid(const Eigen::VectorXd &Th, double scaleFactor)
    {
        // 1) Compute all transforms T0->i (i=0..6)
        std::vector<Eigen::Matrix4d> Tm = computeChainFK(Th, scaleFactor);

        // 2) Initialize a 6x6 Jacobian
        Eigen::Matrix<double, 6, 6> J;
        J.setZero();

        // 3) The end-effector origin in base frame
        Eigen::Vector3d o_6 = Tm[6].block<3,1>(0,3);

        // For each joint i in [1..6]:
        for (int i = 1; i <= 6; ++i)
        {
            // z_{i-1} is the third column of Tm[i-1]'s rotation part
            Eigen::Vector3d z_im1 = Tm[i-1].block<3,3>(0,0).col(2);

            // o_{i-1} is the translation part
            Eigen::Vector3d o_im1 = Tm[i-1].block<3,1>(0,3);

            // Linear part: z_{i-1} x (o_6 - o_{i-1})
            Eigen::Vector3d linPart = z_im1.cross(o_6 - o_im1);

            // Angular part: z_{i-1}
            Eigen::Vector3d angPart = z_im1;

            // Place into the i-th column (0-based => J.col(i-1))
            J.block<3,1>(0, i-1) = linPart;
            J.block<3,1>(3, i-1) = angPart;
        }

        // 4) Check determinant for singularity
        double detJ = J.determinant();
        if (std::fabs(detJ) < 1e-5) {
            RCLCPP_WARN(rclcpp::get_logger("ur5_singAvoid"),
                        "Near a singularity! Determinant: %f", detJ);
            return true;
        }

        // Optionally, you could do SVD and check the smallest singular value:
        // double minSv = J.jacobiSvd().singularValues().minCoeff();
        // if (minSv < 1e-4) ...
        //     ...

        return false;
    }
    // A small helper for one DH transform: from frame i to i+1
    Eigen::Matrix4d Tij(double theta, double alpha, double d, double a)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T <<  std::cos(theta), -std::sin(theta)*std::cos(alpha),  std::sin(theta)*std::sin(alpha),  a*std::cos(theta),
            std::sin(theta),  std::cos(theta)*std::cos(alpha), -std::cos(theta)*std::sin(alpha),  a*std::sin(theta),
            0,                std::sin(alpha),                  std::cos(alpha),                  d,
            0,                0,                                0,                                1;
        return T;
    }

    // Computes base->link i for each i, i.e. Tm[i]
    std::vector<Eigen::Matrix4d> computeChainFK(const Eigen::VectorXd &Th, double scaleFactor)
    {
        // UR5 DH parameters (typical) ...
        //  NOTE: May need sign adjustments or tweaks to match your UR5 definition.
        std::vector<double> A = {0,     -0.425,  -0.3922,  0,     0,     0};
        std::vector<double> D = {0.1625, 0,       0,        0.1333,0.0997,0.0996};
        std::vector<double> ALPHA = {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0};

        // Scale if desired
        for (auto &a : A) a *= scaleFactor;
        for (auto &d : D) d *= scaleFactor;

        // Initialize array of transforms T0->i for i=0..6
        std::vector<Eigen::Matrix4d> Tm(7, Eigen::Matrix4d::Identity());
        // Tm[0] = Identity (base frame)

        // Multiply each successive transform
        for (int i=0; i<6; ++i) {
            Eigen::Matrix4d Ti = Tij(Th(i), ALPHA[i], D[i], A[i]);
            Tm[i+1] = Tm[i] * Ti;  // T0->i+1 = T0->i * Ti
        }

        // Now Tm[6] = T0->6 is the final end-effector
        return Tm;
    }



    // ---------------------------------------------------------------------------
    // 6) The Service Callback: computes waypoints, then generates a cubic trajectory
    // ---------------------------------------------------------------------------
    void compute_trajectory_callback(
        const std::shared_ptr<custom_msg_interfaces::srv::ComputeTrajectory::Request> request,
        std::shared_ptr<custom_msg_interfaces::srv::ComputeTrajectory::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[CALLBACK] compute_trajectory_callback STARTED");
        const size_t num_poses = request->array.poses.size();
        RCLCPP_INFO(this->get_logger(), "Got %zu poses", num_poses);

        // Basic checks
        if (num_poses == 0) {
            RCLCPP_WARN(this->get_logger(), "[WARNING] No poses received!");
            response->trajectory = trajectory_msgs::msg::JointTrajectory();
            response->status_message = "No poses provided";
            return;
        }

        if (!ik_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "IK service not available after waiting");
            response->trajectory = trajectory_msgs::msg::JointTrajectory();
            response->status_message = "IK service unavailable";
            return;
        }

        // -----------------------------------------------------------------------
        // Build the global 'waypoints' with (N+1) rows:
        //   row 0 = initial joints,
        //   row i+1 = best solution for poses[i]
        // -----------------------------------------------------------------------
        waypoints.clear();
        waypoints.resize(num_poses + 1);

        // Fill row 0 with the initial joint array
        for (size_t j = 0; j < 6; ++j) {
            waypoints[0][j] = initial_joint_array_[j];
        }

        // For each pose i: call IK, pick closest solution to waypoints[i]
        for (size_t i = 0; i < num_poses; ++i) {
            RCLCPP_INFO(this->get_logger(), "Processing Pose %zu/%zu", i + 1, num_poses);

            auto ik_request = std::make_shared<custom_msg_interfaces::srv::ComputeIK::Request>();
            ik_request->header.stamp = this->now();
            ik_request->header.frame_id = "base";
            ik_request->target_pose = request->array.poses[i];

            // Call the IK server
            auto future_result = ik_client_->async_send_request(ik_request);
            auto ret = rclcpp::spin_until_future_complete(
                ik_client_node_->get_node_base_interface(),
                future_result, 10s
            );

            if (ret == rclcpp::FutureReturnCode::SUCCESS) {
                auto result = future_result.get();
                if (!result || result->joint_angles_matrix.data.empty()) {
                    RCLCPP_WARN(this->get_logger(), "IK computation for Pose %zu returned no solution!", i + 1);
                } else {
                    RCLCPP_INFO(this->get_logger(), "[SUCCESS] IK response for Pose %zu received", i + 1);
                    print_joint_angles_matrix(result->joint_angles_matrix.data);

                    // Convert waypoints[i] to vector<double>
                    std::vector<double> prev_joints(waypoints[i].begin(), waypoints[i].end());

                    // Pick best row from the 8x6 matrix
                    std::vector<double> best_solution =
                        select_closest_one(prev_joints, result->joint_angles_matrix.data);

                    Eigen::VectorXd Th = Eigen::VectorXd::Map(best_solution.data(), best_solution.size());

                    // Check for singularity using ROS2 logging
                    if (ur5_singAvoid(Th, 1.0)) {
                        RCLCPP_WARN(rclcpp::get_logger("compute_trajectory_service"), "Selected joint configuration is near a singularity!");
                        std::fill(best_solution.begin(), best_solution.end(), std::numeric_limits<double>::quiet_NaN());
                        //this will stop the trajecotry generation and thus the robot
                    }

                    // Store in row i+1
                    for (size_t j = 0; j < 6; ++j) {
                        waypoints[i + 1][j] = best_solution[j];
                    }
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "IK service call for Pose %zu failed!", i + 1);
            }
        }

        // (Optional) Print final global waypoints
        RCLCPP_INFO(this->get_logger(), "WAYPOINTS (total rows = %zu):", waypoints.size());
        for (size_t i = 0; i < waypoints.size(); ++i) {
            std::ostringstream row_stream;
            row_stream << "Row " << i << ": ";
            for (size_t j = 0; j < 6; ++j) {
                row_stream << waypoints[i][j] << (j < 5 ? ", " : "");
            }
            RCLCPP_INFO(this->get_logger(), "%s", row_stream.str().c_str());
        }

        // -----------------------------------------------------------------------
        // 7) Now Generate a Real Trajectory from Those Waypoints (Cubic Spline)
        // -----------------------------------------------------------------------
        double segment_time = 0.8; // [seconds] for each segment, adjust as needed
        trajectory_msgs::msg::JointTrajectory cubic_traj =
            generate_cubic_trajectory(waypoints, segment_time);

        // Send final trajectory in the service response
        response->trajectory = cubic_traj;
        response->status_message = "IK computed, cubic trajectory generated successfully.";
    }
};

// ---------------------------------------------------------------------------
// 8) main(): spin the node
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputeTrajectoryService>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
