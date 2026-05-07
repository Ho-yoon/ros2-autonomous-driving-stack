cmake_minimum_required(VERSION 3.14)
project(av_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(include)

add_library(ekf_localizer SHARED src/ekf_localizer.cpp)
ament_target_dependencies(ekf_localizer
  rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(ekf_localizer Eigen3::Eigen)
rclcpp_components_register_node(ekf_localizer
  PLUGIN "av_localization::EkfLocalizer"
  EXECUTABLE "ekf_localizer_node")

add_library(gnss_imu_fusion SHARED src/gnss_imu_fusion.cpp)
ament_target_dependencies(gnss_imu_fusion
  rclcpp rclcpp_components sensor_msgs geometry_msgs)
target_link_libraries(gnss_imu_fusion ${GeographicLib_LIBRARIES})
rclcpp_components_register_node(gnss_imu_fusion
  PLUGIN "av_localization::GnssImuFusion"
  EXECUTABLE "gnss_imu_fusion_node")

install(TARGETS ekf_localizer gnss_imu_fusion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
