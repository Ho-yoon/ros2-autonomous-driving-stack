#pragma once

#include <string>

#include <av_msgs/msg/trajectory_point_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace av_planning
{

class TrajectoryGenerator : public rclcpp::Node
{
public:
  explicit TrajectoryGenerator(const rclcpp::NodeOptions & options);

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Publisher<av_msgs::msg::TrajectoryPointArray>::SharedPtr pub_traj_;

  std::string behavior_state_;
  double target_speed_;
  double horizon_;
  double spacing_;
};

}  // namespace av_planning
