cmake_minimum_required(VERSION 3.8)
project(tiago_auto_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)

add_executable(move_to_target src/move_to_target.cpp)

ament_target_dependencies(move_to_target 
  rclcpp 
  geometry_msgs 
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp_action
  control_msgs
  trajectory_msgs
  linkattacher_msgs
)

install(TARGETS move_to_target
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
