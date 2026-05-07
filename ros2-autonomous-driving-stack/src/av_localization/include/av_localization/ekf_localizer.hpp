#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace av_localization
{

class EkfLocalizer : public rclcpp::Node
{
public:
  explicit EkfLocalizer(const rclcpp::NodeOptions & options);

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_gnss_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void publish();

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  Eigen::VectorXd state_;       // [x, y, z, roll, pitch, yaw]
  Eigen::MatrixXd covariance_;

  rclcpp::Time last_predict_time_;
  double process_noise_pos_;
  double process_noise_yaw_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace av_localization
