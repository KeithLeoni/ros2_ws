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

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(inverse_kin_server_node src/inverse_kin_server.cpp)
add_executable(direct_kin_server src/direct_kin_server.cpp)

ament_target_dependencies(inverse_kin_server_node
  rclcpp
  custom_msg_interfaces
  geometry_msgs
  std_msgs
  Eigen3
)

ament_target_dependencies(direct_kin_server 
  rclcpp 
  custom_msg_interfaces 
  geometry_msgs
  std_msgs
  Eigen3
)

install(TARGETS
  inverse_kin_server_node
  direct_kin_server
  DESTINATION lib/${PROJECT_NAME}
)

# Export ...
ament_package()
