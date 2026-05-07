cmake_minimum_required(VERSION 3.14)
project(av_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(av_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation search)

include_directories(include)

add_library(object_detector SHARED src/object_detector_node.cpp)
ament_target_dependencies(object_detector
  rclcpp rclcpp_components sensor_msgs geometry_msgs av_msgs pcl_conversions)
target_link_libraries(object_detector ${PCL_LIBRARIES})
rclcpp_components_register_node(object_detector
  PLUGIN "av_perception::ObjectDetectorNode"
  EXECUTABLE "object_detector_node")

add_library(tracking SHARED src/tracking_node.cpp)
ament_target_dependencies(tracking rclcpp rclcpp_components av_msgs)
rclcpp_components_register_node(tracking
  PLUGIN "av_perception::TrackingNode"
  EXECUTABLE "tracking_node")

install(TARGETS object_detector tracking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
