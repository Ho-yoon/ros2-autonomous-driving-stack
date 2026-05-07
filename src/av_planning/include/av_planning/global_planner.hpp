#pragma once

#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace av_planning
{

struct Waypoint
{
  int id;
  double x, y;
  std::vector<int> neighbours;
};

class GlobalPlanner : public rclcpp::Node
{
public:
  explicit GlobalPlanner(const rclcpp::NodeOptions & options);

private:
  void build_demo_graph();
  std::vector<int> dijkstra(int start, int goal);
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

  std::unordered_map<int, Waypoint> waypoints_;
  double waypoint_spacing_;
};

}  // namespace av_planning
