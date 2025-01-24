#include "motion_pkg/inverse_kin_server.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#ifndef M_PI_2 
#define M_PI_2 1.57079632679489661923 
#endif
#ifndef M_PI 
#define M_PI 3.14159265358979323846 
#endif

using namespace std;
using namespace Eigen;
using Eigen::MatrixXf;

namespace motion_pkg
{

// Constants for UR5 (meters, radians)
static const float A[6] = {0.0f, -0.425f, -0.3922f, 0.0f, 0.0f, 0.0f};
static const float D[6] = {0.1625f, 0.0f, 0.0f, 0.1333f, 0.0997f, 0.0996f};

// node
InverseKinServer::InverseKinServer(const rclcpp::NodeOptions & options): Node("inverse_kin_server_node", options)
{
 // Create the service
 using namespace std::placeholders;
 service_ = this->create_service<custom_msg_interfaces::srv::ComputeIK>(
   "compute_ik",
   std::bind(&InverseKinServer::computeIKCallback, this, _1, _2));

 RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Service is ready.");
}

void InverseKinServer::computeIKCallback(
   const std::shared_ptr<custom_msg_interfaces::srv::ComputeIK::Request> request,
   std::shared_ptr<custom_msg_interfaces::srv::ComputeIK::Response> response)
{
 // Extract the target pose from the request
 auto & pose = request->target_pose;

 // Position
 float x = static_cast<float>(pose.position.x);
 float y = static_cast<float>(pose.position.y);
 float z = static_cast<float>(pose.position.z);

 // Orientation (quaternion)
 double qx = pose.orientation.x;
 double qy = pose.orientation.y;
 double qz = pose.orientation.z;
 double qw = pose.orientation.w;

 // Convert quaternion to Eigen rotation matrix (using double precision then cast to float)
 Eigen::Quaterniond q_eig(qw, qx, qy, qz);
 Eigen::Matrix3d R_d = q_eig.normalized().toRotationMatrix(); // 3x3 double
 Eigen::Matrix3f R_f = R_d.cast<float>(); // cast to float

 // Position as Eigen Vector3f
 Eigen::Vector3f p60(x, y, z);

 // For simplicity, let's just assume scale factor = 1.0 (as in your code)
 float scaleFactor = 1.0f;

 // Compute the inverse kinematics
 Eigen::MatrixXd solutions = ur5Inverse(p60, R_f, scaleFactor);  // 8x6

 // Prepare the response multiarray: we have 8 solutions, each with 6 joints
 response->joint_angles_matrix.layout.dim.resize(2);

 // First dimension = number of solutions
 response->joint_angles_matrix.layout.dim[0].label = "solutions";
 response->joint_angles_matrix.layout.dim[0].size = 8;         // we know we always produce 8
 response->joint_angles_matrix.layout.dim[0].stride = 8 * 6;   // each row has 6 entries, total 8 rows

 // Second dimension = number of joints
 response->joint_angles_matrix.layout.dim[1].label = "joints";
 response->joint_angles_matrix.layout.dim[1].size = 6;         // 6 joints
 response->joint_angles_matrix.layout.dim[1].stride = 6;       // stride

 response->joint_angles_matrix.layout.data_offset = 0;
 response->joint_angles_matrix.data.resize(8 * 6);

 // Fill the data in row-major order
 for (int i = 0; i < 8; ++i)
 {
   for (int j = 0; j < 6; ++j)
   {
     // Convert float (or double) to double for the message
     double angle_val = static_cast<double>(solutions(i, j));
     response->joint_angles_matrix.data[i * 6 + j] = angle_val;
   }
 }

 // A simple status message
 response->status_message = "Inverse kinematics solutions computed successfully.";
 RCLCPP_INFO(this->get_logger(), "IK solutions computed and sent back to client.");
}

bool almzero(float x) {
    return abs(x) < 1e-7;
}

Matrix4f Tij(float th, float alpha, float d, float a) {
    Matrix4f T;
    T << cos(th), -sin(th) * cos(alpha), sin(th) * sin(alpha), a * cos(th),
         sin(th), cos(th) * cos(alpha), -cos(th) * sin(alpha), a * sin(th),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1;
    return T;
}

//Eigen::MatrixXd InverseKinServer::ur5Inverse(
Eigen::MatrixXd InverseKinServer::ur5Inverse(const Eigen::Vector3f & p60, const Eigen::Matrix3f & R60, float scaleFactor)
{
   MatrixXd solutions(8, 6); // 8 possible solutions, each with 6 joint angles
    solutions.setConstant(NAN); // Initialize with NaN

    float A_scaled[6], D_scaled[6];
    for (int i = 0; i < 6; ++i) {
        A_scaled[i] = A[i] * scaleFactor;
        D_scaled[i] = D[i] * scaleFactor;
    }
    float Alpha[6] = {M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0};

    Matrix4f T60;
    T60.block<3, 3>(0, 0) = R60;
    T60.block<3, 1>(0, 3) = p60;
    T60.row(3) << 0, 0, 0, 1;

    Vector4f p50 = T60 * Vector4f(0, 0, -D_scaled[5], 1);
    float psi = atan2(p50(1), p50(0));
    float p50xy = hypot(p50(1), p50(0));
    if (p50xy < D_scaled[3]) {
        cerr << "Position request in the unreachable cylinder" << endl;
        return solutions;
    }
    float phi1_1 = acos(D_scaled[3] / p50xy);
    float phi1_2 = -phi1_1;

    float th1_1 = psi + phi1_1 + M_PI / 2;
    float th1_2 = psi + phi1_2 + M_PI / 2;

    float p61z_1 = p60(0) * sin(th1_1) - p60(1) * cos(th1_1);
    float p61z_2 = p60(0) * sin(th1_2) - p60(1) * cos(th1_2);

    float th5_1_1 = acos((p61z_1 - D_scaled[3]) / D_scaled[5]);
    float th5_1_2 = -acos((p61z_1 - D_scaled[3]) / D_scaled[5]);
    float th5_2_1 = acos((p61z_2 - D_scaled[3]) / D_scaled[5]);
    float th5_2_2 = -acos((p61z_2 - D_scaled[3]) / D_scaled[5]);

    Matrix4f T10_1 = Tij(th1_1, Alpha[0], D_scaled[0], A_scaled[0]);
    Matrix4f T10_2 = Tij(th1_2, Alpha[0], D_scaled[0], A_scaled[0]);

    Matrix4f T16_1 = (T10_1.inverse() * T60).inverse();
    Matrix4f T16_2 = (T10_2.inverse() * T60).inverse();

    float zy_1 = T16_1(1, 2);
    float zx_1 = T16_1(0, 2);

    float zy_2 = T16_2(1, 2);
    float zx_2 = T16_2(0, 2);

    float th6_1_1, th6_1_2, th6_2_1, th6_2_2;
    if (almzero(sin(th5_1_1)) || (almzero(zy_1) && almzero(zx_1))) {
        cerr << "Singular configuration. Choosing arbitrary th6" << endl;
        th6_1_1 = 0;
    } else {
        th6_1_1 = atan2((-zy_1 / sin(th5_1_1)), (zx_1 / sin(th5_1_1)));
    }

    if (almzero(sin(th5_1_2)) || (almzero(zy_1) && almzero(zx_1))) {
        cerr << "Singular configuration. Choosing arbitrary th6" << endl;
        th6_1_2 = 0;
    } else {
        th6_1_2 = atan2((-zy_1 / sin(th5_1_2)), (zx_1 / sin(th5_1_2)));
    }

    if (almzero(sin(th5_2_1)) || (almzero(zy_2) && almzero(zx_2))) {
        cerr << "Singular configuration. Choosing arbitrary th6" << endl;
        th6_2_1 = 0;
    } else {
        th6_2_1 = atan2((-zy_2 / sin(th5_2_1)), (zx_2 / sin(th5_2_1)));
    }

    if (almzero(sin(th5_2_2)) || (almzero(zy_2) && almzero(zx_2))) {
        cerr << "Singular configuration. Choosing arbitrary th6" << endl;
        th6_2_2 = 0;
    } else {
        th6_2_2 = atan2((-zy_2 / sin(th5_2_2)), (zx_2 / sin(th5_2_2)));
    }

    Matrix4f T61_1 = T16_1.inverse();
    Matrix4f T61_2 = T16_2.inverse();

    Matrix4f T54_1_1 = Tij(th5_1_1, Alpha[4], D_scaled[4], A_scaled[4]);
    Matrix4f T54_1_2 = Tij(th5_1_2, Alpha[4], D_scaled[4], A_scaled[4]);
    Matrix4f T54_2_1 = Tij(th5_2_1, Alpha[4], D_scaled[4], A_scaled[4]);
    Matrix4f T54_2_2 = Tij(th5_2_2, Alpha[4], D_scaled[4], A_scaled[4]);

    Matrix4f T65_1_1 = Tij(th6_1_1, Alpha[5], D_scaled[5], A_scaled[5]);
    Matrix4f T65_1_2 = Tij(th6_1_2, Alpha[5], D_scaled[5], A_scaled[5]);
    Matrix4f T65_2_1 = Tij(th6_2_1, Alpha[5], D_scaled[5], A_scaled[5]);
    Matrix4f T65_2_2 = Tij(th6_2_2, Alpha[5], D_scaled[5], A_scaled[5]);

    Matrix4f T41_1_1 = T61_1 * (T54_1_1 * T65_1_1).inverse();
    Matrix4f T41_1_2 = T61_1 * (T54_1_2 * T65_1_2).inverse();
    Matrix4f T41_2_1 = T61_2 * (T54_2_1 * T65_2_1).inverse();
    Matrix4f T41_2_2 = T61_2 * (T54_2_2 * T65_2_2).inverse();

    Vector4f P;
    Vector3f P31_1_1, P31_1_2, P31_2_1, P31_2_2;

    P = T41_1_1 * Vector4f(0, -D_scaled[3], 0, 1);
    P31_1_1 = P.head<3>();
    P = T41_1_2 * Vector4f(0, -D_scaled[3], 0, 1);
    P31_1_2 = P.head<3>();
    P = T41_2_1 * Vector4f(0, -D_scaled[3], 0, 1);
    P31_2_1 = P.head<3>();
    P = T41_2_2 * Vector4f(0, -D_scaled[3], 0, 1);
    P31_2_2 = P.head<3>();

    float C;
    float th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2;
    float th3_2_1_1, th3_2_1_2, th3_2_2_1, th3_2_2_2;

    C = (P31_1_1.squaredNorm() - A_scaled[1] * A_scaled[1] - A_scaled[2] * A_scaled[2]) / (2 * A_scaled[1] * A_scaled[2]);
    if (abs(C) > 1) {
        cerr << "Point out of the work space" << endl;
        th3_1_1_1 = NAN;
        th3_1_1_2 = NAN;
    } else {
        th3_1_1_1 = acos(C);
        th3_1_1_2 = -acos(C);
    }

    C = (P31_1_2.squaredNorm() - A_scaled[1] * A_scaled[1] - A_scaled[2] * A_scaled[2]) / (2 * A_scaled[1] * A_scaled[2]);
    if (abs(C) > 1) {
        cerr << "Point out of the work space" << endl;
        th3_1_2_1 = NAN;
        th3_1_2_2 = NAN;
    } else {
        th3_1_2_1 = acos(C);
        th3_1_2_2 = -acos(C);
    }

    C = (P31_2_1.squaredNorm() - A_scaled[1] * A_scaled[1] - A_scaled[2] * A_scaled[2]) / (2 * A_scaled[1] * A_scaled[2]);
    if (abs(C) > 1) {
        cerr << "Point out of the work space" << endl;
        th3_2_1_1 = NAN;
        th3_2_1_2 = NAN;
    } else {
        th3_2_1_1 = acos(C);
        th3_2_1_2 = -acos(C);
    }

    C = (P31_2_2.squaredNorm() - A_scaled[1] * A_scaled[1] - A_scaled[2] * A_scaled[2]) / (2 * A_scaled[1] * A_scaled[2]);
    if (abs(C) > 1) {
        cerr << "Point out of the work space" << endl;
        th3_2_2_1 = NAN;
        th3_2_2_2 = NAN;
    } else {
        th3_2_2_1 = acos(C);
        th3_2_2_2 = -acos(C);
    }

    float th2_1_1_1 = -atan2(P31_1_1(1), -P31_1_1(0)) + asin((A_scaled[2] * sin(th3_1_1_1)) / P31_1_1.norm());
    float th2_1_1_2 = -atan2(P31_1_1(1), -P31_1_1(0)) + asin((A_scaled[2] * sin(th3_1_1_2)) / P31_1_1.norm());
    float th2_1_2_1 = -atan2(P31_1_2(1), -P31_1_2(0)) + asin((A_scaled[2] * sin(th3_1_2_1)) / P31_1_2.norm());
    float th2_1_2_2 = -atan2(P31_1_2(1), -P31_1_2(0)) + asin((A_scaled[2] * sin(th3_1_2_2)) / P31_1_2.norm());
    float th2_2_1_1 = -atan2(P31_2_1(1), -P31_2_1(0)) + asin((A_scaled[2] * sin(th3_2_1_1)) / P31_2_1.norm());
    float th2_2_1_2 = -atan2(P31_2_1(1), -P31_2_1(0)) + asin((A_scaled[2] * sin(th3_2_1_2)) / P31_2_1.norm());
    float th2_2_2_1 = -atan2(P31_2_2(1), -P31_2_2(0)) + asin((A_scaled[2] * sin(th3_2_2_1)) / P31_2_2.norm());
    float th2_2_2_2 = -atan2(P31_2_2(1), -P31_2_2(0)) + asin((A_scaled[2] * sin(th3_2_2_2)) / P31_2_2.norm());

    Matrix4f T21, T32, T41, T43;
    float xy, xx, th4_1_1_1, th4_1_1_2, th4_1_2_1, th4_1_2_2;
    float th4_2_1_1, th4_2_1_2, th4_2_2_1, th4_2_2_2;

    T21 = Tij(th2_1_1_1, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_1_1_1, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_1_1;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_1_1_1 = atan2(xy, xx);

    T21 = Tij(th2_1_1_2, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_1_1_2, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_1_1;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_1_1_2 = atan2(xy, xx);

    T21 = Tij(th2_1_2_1, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_1_2_1, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_1_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_1_2_1 = atan2(xy, xx);

    T21 = Tij(th2_1_2_2, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_1_2_2, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_1_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_1_2_2 = atan2(xy, xx);
    
    //4_2_...

    T21 = Tij(th2_2_1_1, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_2_1_1, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_2_1;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_2_1_1 = atan2(xy, xx);

    T21 = Tij(th2_2_1_2, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_2_1_2, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_2_1;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_2_1_2 = atan2(xy, xx);
    

    T21 = Tij(th2_2_2_1, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_2_2_1, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_2_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_2_2_1 = atan2(xy, xx);
    
    T21 = Tij(th2_2_2_2, Alpha[1], D_scaled[1], A_scaled[1]);
    T32 = Tij(th3_2_2_2, Alpha[2], D_scaled[2], A_scaled[2]);
    T41 = T41_2_2;
    T43 = (T21 * T32).inverse() * T41;
    xy = T43(1, 0);
    xx = T43(0, 0);
    th4_2_2_2 = atan2(xy, xx);

    // Store the solutions in the MatrixXd
    solutions.row(0) << th1_1, th2_1_1_1, th3_1_1_1, th4_1_1_1, th5_1_1, th6_1_1;
    solutions.row(1) << th1_1, th2_1_1_2, th3_1_1_2, th4_1_1_2, th5_1_1, th6_1_1;
    solutions.row(2) << th1_1, th2_1_2_1, th3_1_2_1, th4_1_2_1, th5_1_2, th6_1_2;
    solutions.row(3) << th1_1, th2_1_2_2, th3_1_2_2, th4_1_2_2, th5_1_2, th6_1_2;
    solutions.row(4) << th1_2, th2_2_1_1, th3_2_1_1, th4_2_1_1, th5_2_1, th6_2_1;
    solutions.row(5) << th1_2, th2_2_1_2, th3_2_1_2, th4_2_1_2, th5_2_1, th6_2_1;
    solutions.row(6) << th1_2, th2_2_2_1, th3_2_2_1, th4_2_2_1, th5_2_2, th6_2_2;
    solutions.row(7) << th1_2, th2_2_2_2, th3_2_2_2, th4_2_2_2, th5_2_2, th6_2_2;

    return solutions;


}
}// namespace motion_pkg

// A simple main to spin this server node if you want a standalone executable
int main(int argc, char ** argv)
{
 rclcpp::init(argc, argv);
 rclcpp::spin(std::make_shared<motion_pkg::InverseKinServer>());
 rclcpp::shutdown();
 return 0;
}



