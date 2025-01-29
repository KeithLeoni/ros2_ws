cmake_minimum_required(VERSION 3.8)
project(motion_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


add_executable(inverse_kin_server_node src/inverse_kin_server.cpp)
ament_target_dependencies(inverse_kin_server_node
  rclcpp
  custom_msg_interfaces
  geometry_msgs
  std_msgs
  Eigen3
)
add_executable(direct_kin_server src/direct_kin_server.cpp)
ament_target_dependencies(direct_kin_server 
  rclcpp 
  custom_msg_interfaces 
  geometry_msgs
  std_msgs
  Eigen3
)

add_executable(compute_trajectory_service src/compute_trajectory_service.cpp)
ament_target_dependencies(compute_trajectory_service
  rclcpp
  control_msgs
  rclcpp_action
  trajectory_msgs
  std_msgs
  geometry_msgs
  custom_msg_interfaces
  std_srvs
  Eigen3
  sensor_msgs
  visualization_msgs
  tf2 
  tf2_geometry_msgs
)


add_executable(compute_path_service src/compute_path_service.cpp)
ament_target_dependencies(compute_path_service 
  rclcpp 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs 
  custom_msg_interfaces
)

install(TARGETS
  inverse_kin_server_node
  direct_kin_server
  compute_trajectory_service
  compute_path_service
  DESTINATION lib/${PROJECT_NAME}
)

# Export ...
ament_package()
