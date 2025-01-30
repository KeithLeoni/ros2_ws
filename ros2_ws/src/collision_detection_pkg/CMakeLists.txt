cmake_minimum_required(VERSION 3.8)
project(collision_detection_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(table_avoidance_node src/table_avoidance_node.cpp)
ament_target_dependencies(table_avoidance_node rclcpp std_msgs Eigen3)

install(TARGETS
  table_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
