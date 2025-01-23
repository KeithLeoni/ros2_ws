//#include "motion_pkg/inverse_kin_server.hpp"
//
//// ROS2
//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/pose.hpp"
//#include "std_msgs/msg/float64_multi_array.hpp"
//#include "builtin_interfaces/msg/time.hpp"

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
using Eigen::MatrixXf;



using namespace std;
using namespace Eigen;

//namespace motion_pkg
//{
//
//// Constants for UR5 (meters, radians)
//static const float A[6] = {0.0f, -0.425f, -0.3922f, 0.0f, 0.0f, 0.0f};
//static const float D[6] = {0.1625f, 0.0f, 0.0f, 0.1333f, 0.0997f, 0.0996f};
//
//InverseKinServer::InverseKinServer(const rclcpp::NodeOptions & options)
//: Node("inverse_kin_server_node", options)
//{
//  // Create the service
//  using namespace std::placeholders;
//  service_ = this->create_service<custom_msg_interfaces::srv::ComputeIK>(
//    "compute_ik",
//    std::bind(&InverseKinServer::computeIKCallback, this, _1, _2));
//
//  RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Service is ready.");
//}
//
//void InverseKinServer::computeIKCallback(
//    const std::shared_ptr<custom_msg_interfaces::srv::ComputeIK::Request> request,
//    std::shared_ptr<custom_msg_interfaces::srv::ComputeIK::Response> response)
//{
//  // Extract the target pose from the request
//  auto & pose = request->target_pose;
//
//  // Position
//  float x = static_cast<float>(pose.position.x);
//  float y = static_cast<float>(pose.position.y);
//  float z = static_cast<float>(pose.position.z);
//
//  // Orientation (quaternion)
//  double qx = pose.orientation.x;
//  double qy = pose.orientation.y;
//  double qz = pose.orientation.z;
//  double qw = pose.orientation.w;
//
//  // Convert quaternion to Eigen rotation matrix (using double precision then cast to float)
//  Eigen::Quaterniond q_eig(qw, qx, qy, qz);
//  Eigen::Matrix3d R_d = q_eig.normalized().toRotationMatrix(); // 3x3 double
//  Eigen::Matrix3f R_f = R_d.cast<float>(); // cast to float
//
//  // Position as Eigen Vector3f
//  Eigen::Vector3f p60(x, y, z);
//
//  // For simplicity, let's just assume scale factor = 1.0 (as in your code)
//  float scaleFactor = 1.0f;
//
//  // Compute the inverse kinematics
//  Eigen::MatrixXd solutions = ur5Inverse(p60, R_f, scaleFactor);  // 8x6
//
//  // Prepare the response multiarray: we have 8 solutions, each with 6 joints
//  response->joint_angles_matrix.layout.dim.resize(2);
//
//  // First dimension = number of solutions
//  response->joint_angles_matrix.layout.dim[0].label = "solutions";
//  response->joint_angles_matrix.layout.dim[0].size = 8;         // we know we always produce 8
//  response->joint_angles_matrix.layout.dim[0].stride = 8 * 6;   // each row has 6 entries, total 8 rows
//
//  // Second dimension = number of joints
//  response->joint_angles_matrix.layout.dim[1].label = "joints";
//  response->joint_angles_matrix.layout.dim[1].size = 6;         // 6 joints
//  response->joint_angles_matrix.layout.dim[1].stride = 6;       // stride
//
//  response->joint_angles_matrix.layout.data_offset = 0;
//  response->joint_angles_matrix.data.resize(8 * 6);
//
//  // Fill the data in row-major order
//  for (int i = 0; i < 8; ++i)
//  {
//    for (int j = 0; j < 6; ++j)
//    {
//      // Convert float (or double) to double for the message
//      double angle_val = static_cast<double>(solutions(i, j));
//      response->joint_angles_matrix.data[i * 6 + j] = angle_val;
//    }
//  }
//
//  // A simple status message
//  response->status_message = "Inverse kinematics solutions computed successfully.";
//  RCLCPP_INFO(this->get_logger(), "IK solutions computed and sent back to client.");
//}

//Eigen::MatrixXd InverseKinServer::ur5Inverse(
Eigen::MatrixXd ur5Inverse(
    const Eigen::Vector3f & p60, 
    const Eigen::Matrix3f & R60, 
    float scaleFactor)
{
	
	//// Constants for UR5 (meters, radians)
static const float A[6] = {0.0f, -0.425f, -0.3922f, 0.0f, 0.0f, 0.0f};
static const float D[6] = {0.1625f, 0.0f, 0.0f, 0.1333f, 0.0997f, 0.0996f};

  // 8 possible solutions, each with 6 joint angles
  // We'll store them in a 8x6 MatrixXd
  Eigen::MatrixXd solutions(8, 6);
  solutions.setConstant(std::numeric_limits<double>::quiet_NaN());

  // Scale the robot constants if needed
  float A_scaled[6], D_scaled[6];
  for (int i = 0; i < 6; ++i)
  {
    A_scaled[i] = A[i] * scaleFactor;
    D_scaled[i] = D[i] * scaleFactor;
  }

  float Alpha[6] = {static_cast<float>(M_PI / 2),
                    0.0f,
                    0.0f,
                    static_cast<float>(M_PI / 2),
                    static_cast<float>(-M_PI / 2),
                    0.0f};

  // Construct T60 from R60 and p60
  Eigen::Matrix4f T60 = Eigen::Matrix4f::Identity();
  T60.block<3,3>(0,0) = R60;
  T60.block<3,1>(0,3) = p60;

  // Helper function to build a standard DH transform
  auto Tij = [&](float th, float alpha, float d, float a) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float cth = std::cos(th);
    float sth = std::sin(th);
    float ca  = std::cos(alpha);
    float sa  = std::sin(alpha);

    T << cth, -sth * ca,  sth * sa, a * cth,
         sth,  cth * ca, -cth * sa, a * sth,
         0.0f,     sa   ,     ca   , d     ,
         0.0f,   0.0f   ,     0.0f  , 1.0f  ;

    return T;
  };

  // Compute p50
  // p50 = T60 * [0, 0, -D_scaled[5], 1]
  Eigen::Vector4f p50 = T60 * Eigen::Vector4f(0.0f, 0.0f, -D_scaled[5], 1.0f);

  // First joint angles
  float psi   = std::atan2(p50(1), p50(0));
  float p50xy = std::hypot(p50(0), p50(1));

  // Check if inside unreachable cylinder
  if (p50xy < D_scaled[3])
  {
    // If unreachable, we simply store solutions with NaN
    //RCLCPP_ERROR(this->get_logger(), "Position request is in the unreachable cylinder region.");
    return solutions;
  }

  float phi1_1 = std::acos(D_scaled[3] / p50xy);
  float phi1_2 = -phi1_1;

  float th1_1 = psi + phi1_1 + static_cast<float>(M_PI / 2);
  float th1_2 = psi + phi1_2 + static_cast<float>(M_PI / 2);

  // Compute possible th5
  auto get_th5 = [&](float th1) {
    float p61z = p60(0) * std::sin(th1) - p60(1) * std::cos(th1);
    // (p61z - D_scaled[3]) / D_scaled[5]
    float val = (p61z - D_scaled[3]) / D_scaled[5];
    if (std::fabs(val) > 1.0f)
    {
      // out of domain for acos
      return std::vector<float>{NAN, NAN};
    }
    else
    {
      float a = std::acos(val);
      return std::vector<float>{ a, -a };
    }
  };

  // For each th1, compute the corresponding two th5 solutions
  auto th5_1s = get_th5(th1_1); // [th5_1_1, th5_1_2]
  auto th5_2s = get_th5(th1_2); // [th5_2_1, th5_2_2]

  // We'll compute T10_1, T10_2
  Eigen::Matrix4f T10_1 = Tij(th1_1, Alpha[0], D_scaled[0], A_scaled[0]);
  Eigen::Matrix4f T10_2 = Tij(th1_2, Alpha[0], D_scaled[0], A_scaled[0]);

  // T16 = (T10^-1 * T60)^-1
  Eigen::Matrix4f T16_1 = (T10_1.inverse() * T60).inverse();
  Eigen::Matrix4f T16_2 = (T10_2.inverse() * T60).inverse();

  // from T16, we get needed elements for th6
  auto compute_th6 = [&](const Eigen::Matrix4f & T16, float th5) {
    float zy = T16(1,2);
    float zx = T16(0,2);

    // Avoid near-zero denominators
//    if (almzero(std::sin(th5)) || (almzero(zy) && almzero(zx)))
//    {
//      RCLCPP_WARN(this->get_logger(), "Singular configuration => setting th6 = 0 arbitrarily.");
//      return 0.0f;
//    }
//    else
//    {
      return std::atan2(-zy / std::sin(th5), zx / std::sin(th5));
//    }
  };

  float th6_1_1 = compute_th6(T16_1, th5_1s[0]);
  float th6_1_2 = compute_th6(T16_1, th5_1s[1]);
  float th6_2_1 = compute_th6(T16_2, th5_2s[0]);
  float th6_2_2 = compute_th6(T16_2, th5_2s[1]);

  // Next we compute T41 for each set by T61^-1 * (T54 * T65)^-1
  Eigen::Matrix4f T61_1 = T16_1.inverse();
  Eigen::Matrix4f T61_2 = T16_2.inverse();

  // T54
  auto T54 = [&](float th5_val) {
    return Tij(th5_val, Alpha[4], D_scaled[4], A_scaled[4]);
  };
  // T65
  auto T65 = [&](float th6_val) {
    return Tij(th6_val, Alpha[5], D_scaled[5], A_scaled[5]);
  };

  // We'll define a small helper to compute T41 = T61 * (T54 * T65).inverse()
  auto compute_T41 = [&](const Eigen::Matrix4f & T61, float th5_val, float th6_val) {
    Eigen::Matrix4f tmp = T54(th5_val) * T65(th6_val);
    return T61 * tmp.inverse();
  };

  Eigen::Matrix4f T41_1_1 = compute_T41(T61_1, th5_1s[0], th6_1_1);
  Eigen::Matrix4f T41_1_2 = compute_T41(T61_1, th5_1s[1], th6_1_2);
  Eigen::Matrix4f T41_2_1 = compute_T41(T61_2, th5_2s[0], th6_2_1);
  Eigen::Matrix4f T41_2_2 = compute_T41(T61_2, th5_2s[1], th6_2_2);

  // Next, compute P31 = T41 * [0, -D_scaled[3], 0, 1]
  auto getP31 = [&](const Eigen::Matrix4f & T41) {
    Eigen::Vector4f v(0.0f, -D_scaled[3], 0.0f, 1.0f);
    Eigen::Vector4f r = T41 * v;
    return r.head<3>();
  };

  Eigen::Vector3f P31_1_1 = getP31(T41_1_1);
  Eigen::Vector3f P31_1_2 = getP31(T41_1_2);
  Eigen::Vector3f P31_2_1 = getP31(T41_2_1);
  Eigen::Vector3f P31_2_2 = getP31(T41_2_2);

  // Helper to compute joint 3 from P31
  auto compute_th3 = [&](const Eigen::Vector3f & P31_val) {
    float C = (P31_val.squaredNorm() - A_scaled[1]*A_scaled[1] - A_scaled[2]*A_scaled[2]) 
               / (2.0f * A_scaled[1] * A_scaled[2]);
    // Check if within [-1,1]
    if (std::fabs(C) > 1.0f) {
//      RCLCPP_ERROR(this->get_logger(), "Point out of workspace. C = %f out of [-1,1].", C);
      return std::vector<float>{NAN, NAN};
    }
    float a = std::acos(C);
    return std::vector<float>{a, -a};
  };

  auto th3_1_1s = compute_th3(P31_1_1); // yields 2 possible solutions
  auto th3_1_2s = compute_th3(P31_1_2);
  auto th3_2_1s = compute_th3(P31_2_1);
  auto th3_2_2s = compute_th3(P31_2_2);

  // Helper for computing th2
  auto compute_th2 = [&](const Eigen::Vector3f & P31_val, float th3_val) {
    float normP = P31_val.norm();
    // -atan2(P31(1), -P31(0)) + asin( (A2 * sin(th3)) / |P31| )
    float angle = -std::atan2(P31_val(1), -P31_val(0)) 
                  + std::asin( (A_scaled[2] * std::sin(th3_val)) / normP );
    return angle;
  };

  float th2_1_1_1 = compute_th2(P31_1_1, th3_1_1s[0]);
  float th2_1_1_2 = compute_th2(P31_1_1, th3_1_1s[1]);
  float th2_1_2_1 = compute_th2(P31_1_2, th3_1_2s[0]);
  float th2_1_2_2 = compute_th2(P31_1_2, th3_1_2s[1]);
  float th2_2_1_1 = compute_th2(P31_2_1, th3_2_1s[0]);
  float th2_2_1_2 = compute_th2(P31_2_1, th3_2_1s[1]);
  float th2_2_2_1 = compute_th2(P31_2_2, th3_2_2s[0]);
  float th2_2_2_2 = compute_th2(P31_2_2, th3_2_2s[1]);

  // Now compute th4 from T43 = (T21 * T32).inverse() * T41
  auto compute_th4 = [&](float th2_val, float th3_val, const Eigen::Matrix4f & T41) {
    Eigen::Matrix4f T21 = Tij(th2_val, Alpha[1], D_scaled[1], A_scaled[1]);
    Eigen::Matrix4f T32 = Tij(th3_val, Alpha[2], D_scaled[2], A_scaled[2]);

    Eigen::Matrix4f T43 = (T21 * T32).inverse() * T41;
    float xy = T43(1, 0);
    float xx = T43(0, 0);
    return std::atan2(xy, xx);
  };

  // We'll fill solutions in the matrix now
  // 0) [th1_1, th2_1_1_1, th3_1_1_1, th4_1_1_1, th5_1_1, th6_1_1]
  {
    float th4_val = compute_th4(th2_1_1_1, th3_1_1s[0], T41_1_1);
    solutions(0,0) = th1_1;
    solutions(0,1) = th2_1_1_1;
    solutions(0,2) = th3_1_1s[0];
    solutions(0,3) = th4_val;
    solutions(0,4) = th5_1s[0];
    solutions(0,5) = th6_1_1;
  }
  // 1)
  {
    float th4_val = compute_th4(th2_1_1_2, th3_1_1s[1], T41_1_1);
    solutions(1,0) = th1_1;
    solutions(1,1) = th2_1_1_2;
    solutions(1,2) = th3_1_1s[1];
    solutions(1,3) = th4_val;
    solutions(1,4) = th5_1s[0];
    solutions(1,5) = th6_1_1;
  }
  // 2)
  {
    float th4_val = compute_th4(th2_1_2_1, th3_1_2s[0], T41_1_2);
    solutions(2,0) = th1_1;
    solutions(2,1) = th2_1_2_1;
    solutions(2,2) = th3_1_2s[0];
    solutions(2,3) = th4_val;
    solutions(2,4) = th5_1s[1];
    solutions(2,5) = th6_1_2;
  }
  // 3)
  {
    float th4_val = compute_th4(th2_1_2_2, th3_1_2s[1], T41_1_2);
    solutions(3,0) = th1_1;
    solutions(3,1) = th2_1_2_2;
    solutions(3,2) = th3_1_2s[1];
    solutions(3,3) = th4_val;
    solutions(3,4) = th5_1s[1];
    solutions(3,5) = th6_1_2;
  }
  // 4)
  {
    float th4_val = compute_th4(th2_2_1_1, th3_2_1s[0], T41_2_1);
    solutions(4,0) = th1_2;
    solutions(4,1) = th2_2_1_1;
    solutions(4,2) = th3_2_1s[0];
    solutions(4,3) = th4_val;
    solutions(4,4) = th5_2s[0];
    solutions(4,5) = th6_2_1;
  }
  // 5)
  {
    float th4_val = compute_th4(th2_2_1_2, th3_2_1s[1], T41_2_1);
    solutions(5,0) = th1_2;
    solutions(5,1) = th2_2_1_2;
    solutions(5,2) = th3_2_1s[1];
    solutions(5,3) = th4_val;
    solutions(5,4) = th5_2s[0];
    solutions(5,5) = th6_2_1;
  }
  // 6)
  {
    float th4_val = compute_th4(th2_2_2_1, th3_2_2s[0], T41_2_2);
    solutions(6,0) = th1_2;
    solutions(6,1) = th2_2_2_1;
    solutions(6,2) = th3_2_2s[0];
    solutions(6,3) = th4_val;
    solutions(6,4) = th5_2s[1];
    solutions(6,5) = th6_2_2;
  }
  // 7)
  {
    float th4_val = compute_th4(th2_2_2_2, th3_2_2s[1], T41_2_2);
    solutions(7,0) = th1_2;
    solutions(7,1) = th2_2_2_2;
    solutions(7,2) = th3_2_2s[1];
    solutions(7,3) = th4_val;
    solutions(7,4) = th5_2s[1];
    solutions(7,5) = th6_2_2;
  }

  return solutions;
}
// namespace motion_pkg
// A simple main to spin this server node if you want a standalone executable
//int main(int argc, char ** argv)
//{
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<motion_pkg::InverseKinServer>());
//  rclcpp::shutdown();
//  return 0;
//}



int main() {
    // Define the position vector and rotation matrix
    Vector3f p60(-0.143, -0.436, 0.206); 
    Matrix3f R60;
//    R60 = AngleAxisf(M_PI / 4, Vector3f::UnitZ()) *
//          AngleAxisf(M_PI / 6, Vector3f::UnitY()) *
//          AngleAxisf(M_PI / 3, Vector3f::UnitX()); 
//          
    //R60 = Eigen::Matrix3f::Identity();
    
    Quaternionf q60(0.015, 0.000, 1.000, 0.008); // Identity quaternion
    q60.normalize();
    R60 = q60.toRotationMatrix(); // Convert quaternion to rotation matrix

    float scaleFactor = 1.0; // Example scale factor

    // Call the ur5Inverse function
    MatrixXd solutions = ur5Inverse(p60, R60, scaleFactor);

    // Display the solutions
    cout << "The joint angles are:" << endl;
    cout << solutions << endl;

    return 0;
}
