cmake_minimum_required(VERSION 3.8)
project(custom_msg_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionResult.msg"
  "srv/ComputeIK.srv"
  "srv/ComputeDirKin.srv"
  "srv/ComputePath.srv"
  "srv/ComputeTrajectory.srv"
  "srv/MoveAB.srv"
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs

)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
