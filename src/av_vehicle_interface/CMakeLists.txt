cmake_minimum_required(VERSION 3.14)
project(av_vehicle_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(av_msgs REQUIRED)

include_directories(include)

add_library(vehicle_command_adapter SHARED src/vehicle_command_adapter.cpp)
ament_target_dependencies(vehicle_command_adapter
  rclcpp rclcpp_components geometry_msgs std_msgs av_msgs)
rclcpp_components_register_node(vehicle_command_adapter
  PLUGIN "av_vehicle_interface::VehicleCommandAdapter"
  EXECUTABLE "vehicle_command_adapter_node")

add_library(can_gateway SHARED src/can_gateway_node.cpp)
ament_target_dependencies(can_gateway rclcpp rclcpp_components av_msgs)
rclcpp_components_register_node(can_gateway
  PLUGIN "av_vehicle_interface::CanGatewayNode"
  EXECUTABLE "can_gateway_node")

install(TARGETS vehicle_command_adapter can_gateway
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
