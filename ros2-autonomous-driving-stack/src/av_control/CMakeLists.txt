cmake_minimum_required(VERSION 3.14)
project(av_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(av_msgs REQUIRED)

include_directories(include)

add_library(pure_pursuit_controller SHARED src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller
  rclcpp rclcpp_components geometry_msgs nav_msgs std_msgs av_msgs)
rclcpp_components_register_node(pure_pursuit_controller
  PLUGIN "av_control::PurePursuitController"
  EXECUTABLE "pure_pursuit_node")

add_library(pid_speed_controller SHARED src/pid_speed_controller.cpp)
ament_target_dependencies(pid_speed_controller
  rclcpp rclcpp_components geometry_msgs nav_msgs av_msgs)
rclcpp_components_register_node(pid_speed_controller
  PLUGIN "av_control::PidSpeedController"
  EXECUTABLE "pid_speed_node")

install(TARGETS pure_pursuit_controller pid_speed_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
