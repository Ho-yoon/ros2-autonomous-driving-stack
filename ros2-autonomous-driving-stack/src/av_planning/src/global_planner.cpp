#include "av_planning/global_planner.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>

#include <av_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace av_planning
{

GlobalPlanner::GlobalPlanner(const rclcpp::NodeOptions & options)
: Node("global_planner", options)
{
  declare_parameter("waypoint_spacing", 1.0);
  waypoint_spacing_ = get_parameter("waypoint_spacing").as_double();

  auto qos_pose = rclcpp::QoS(1).reliable().transient_local();
  auto qos_rel = rclcpp::QoS(10).reliable().volatile_durability();

  sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose",
    qos_pose,
    std::bind(&GlobalPlanner::on_pose, this, std::placeholders::_1));

  pub_path_ = create_publisher<nav_msgs::msg::Path>("/planning/global_path", qos_rel);

  // Build a simple demo lane graph (straight lane for testing)
  build_demo_graph();

  RCLCPP_INFO(get_logger(), "GlobalPlanner ready");
}

void GlobalPlanner::build_demo_graph()
{
  // 20 waypoints along x-axis, 5 m apart
  for (int i = 0; i < 20; ++i) {
    Waypoint wp;
    wp.id = i;
    wp.x = i * 5.0;
    wp.y = 0.0;
    if (i + 1 < 20) wp.neighbours = {i + 1};
    waypoints_[i] = wp;
  }
}

std::vector<int> GlobalPlanner::dijkstra(int start, int goal)
{
  std::unordered_map<int, double> dist;
  std::unordered_map<int, int> prev;
  for (auto & [id, _] : waypoints_) dist[id] = std::numeric_limits<double>::infinity();
  dist[start] = 0.0;

  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
  pq.push({0.0, start});

  while (!pq.empty()) {
    auto [d, u] = pq.top(); pq.pop();
    if (d > dist[u]) continue;
    if (u == goal) break;
    for (int v : waypoints_[u].neighbours) {
      double dx = waypoints_[v].x - waypoints_[u].x;
      double dy = waypoints_[v].y - waypoints_[u].y;
      double w = std::sqrt(dx * dx + dy * dy);
      if (dist[u] + w < dist[v]) {
        dist[v] = dist[u] + w;
        prev[v] = u;
        pq.push({dist[v], v});
      }
    }
  }

  std::vector<int> path;
  for (int cur = goal; prev.count(cur); cur = prev[cur]) path.push_back(cur);
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

void GlobalPlanner::on_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (waypoints_.empty()) return;

  // Find nearest waypoint to current position
  double min_dist = std::numeric_limits<double>::infinity();
  int nearest = 0;
  for (auto & [id, wp] : waypoints_) {
    double dx = wp.x - msg->pose.pose.position.x;
    double dy = wp.y - msg->pose.pose.position.y;
    double d = std::sqrt(dx * dx + dy * dy);
    if (d < min_dist) { min_dist = d; nearest = id; }
  }

  int goal = static_cast<int>(waypoints_.size()) - 1;
  auto path_ids = dijkstra(nearest, goal);

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = "map";
  for (int id : path_ids) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x = waypoints_[id].x;
    ps.pose.position.y = waypoints_[id].y;
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }
  pub_path_->publish(path_msg);
}

}  // namespace av_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_planning::GlobalPlanner)
