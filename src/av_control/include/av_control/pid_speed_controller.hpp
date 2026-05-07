#pragma once

#include <av_msgs/msg/trajectory_point_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace av_control
{

class PidSpeedController : public rclcpp::Node
{
public:
  explicit PidSpeedController(const rclcpp::NodeOptions & options);

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<av_msgs::msg::TrajectoryPointArray>::SharedPtr sub_traj_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

  av_msgs::msg::TrajectoryPointArray::SharedPtr trajectory_;

  double kp_, ki_, kd_;
  double max_accel_, max_decel_;
  double integral_, prev_error_;
  rclcpp::Time prev_time_;
};

}  // namespace av_control
