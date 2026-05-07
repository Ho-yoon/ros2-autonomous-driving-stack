cmake_minimum_required(VERSION 3.14)
project(av_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(av_msgs REQUIRED)

include_directories(include)

add_library(global_planner SHARED src/global_planner.cpp)
ament_target_dependencies(global_planner
  rclcpp rclcpp_components geometry_msgs nav_msgs av_msgs)
rclcpp_components_register_node(global_planner
  PLUGIN "av_planning::GlobalPlanner"
  EXECUTABLE "global_planner_node")

add_library(behavior_planner SHARED src/behavior_planner.cpp)
ament_target_dependencies(behavior_planner
  rclcpp rclcpp_components geometry_msgs std_msgs av_msgs)
rclcpp_components_register_node(behavior_planner
  PLUGIN "av_planning::BehaviorPlanner"
  EXECUTABLE "behavior_planner_node")

add_library(trajectory_generator SHARED src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator
  rclcpp rclcpp_components nav_msgs std_msgs av_msgs)
rclcpp_components_register_node(trajectory_generator
  PLUGIN "av_planning::TrajectoryGenerator"
  EXECUTABLE "trajectory_generator_node")

install(TARGETS global_planner behavior_planner trajectory_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
