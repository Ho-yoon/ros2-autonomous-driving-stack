cmake_minimum_required(VERSION 3.14)
project(av_sensing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(include)

# LiDAR preprocessor component
add_library(lidar_preprocessor SHARED src/lidar_preprocessor.cpp)
ament_target_dependencies(lidar_preprocessor
  rclcpp rclcpp_components sensor_msgs pcl_conversions pcl_ros)
target_link_libraries(lidar_preprocessor ${PCL_LIBRARIES})
rclcpp_components_register_node(lidar_preprocessor
  PLUGIN "av_sensing::LidarPreprocessor"
  EXECUTABLE "lidar_preprocessor_node")

# Camera preprocessor component
add_library(camera_preprocessor SHARED src/camera_preprocessor.cpp)
ament_target_dependencies(camera_preprocessor
  rclcpp rclcpp_components sensor_msgs cv_bridge)
target_link_libraries(camera_preprocessor ${OpenCV_LIBS})
rclcpp_components_register_node(camera_preprocessor
  PLUGIN "av_sensing::CameraPreprocessor"
  EXECUTABLE "camera_preprocessor_node")

install(TARGETS lidar_preprocessor camera_preprocessor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
