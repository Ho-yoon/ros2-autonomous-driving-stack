#pragma once

#include <av_msgs/msg/trajectory_point_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace av_control
{

class PurePursuitController : public rclcpp::Node
{
public:
  explicit PurePursuitController(const rclcpp::NodeOptions & options);

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  double yaw_from_quat(const geometry_msgs::msg::Quaternion & q) const;

  rclcpp::Subscription<av_msgs::msg::TrajectoryPointArray>::SharedPtr sub_traj_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steering_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_error_;

  av_msgs::msg::TrajectoryPointArray::SharedPtr trajectory_;
  double lookahead_;
  double wheelbase_;
  double max_steer_;
  double min_speed_stanley_;
};

}  // namespace av_control
