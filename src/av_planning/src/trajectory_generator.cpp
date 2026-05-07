#include "av_planning/trajectory_generator.hpp"

#include <cmath>

#include <av_msgs/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>

namespace av_planning
{

TrajectoryGenerator::TrajectoryGenerator(const rclcpp::NodeOptions & options)
: Node("trajectory_generator", options)
{
  declare_parameter("target_speed", 8.0);      // m/s
  declare_parameter("horizon_length", 50.0);   // metres
  declare_parameter("point_spacing", 1.0);     // metres

  target_speed_ = get_parameter("target_speed").as_double();
  horizon_ = get_parameter("horizon_length").as_double();
  spacing_ = get_parameter("point_spacing").as_double();

  auto qos_rel  = rclcpp::QoS(10).reliable().volatile_durability();
  auto qos_traj = rclcpp::QoS(1).reliable().transient_local();

  sub_path_ = create_subscription<nav_msgs::msg::Path>(
    "/planning/global_path", qos_rel,
    std::bind(&TrajectoryGenerator::on_path, this, std::placeholders::_1));

  sub_state_ = create_subscription<std_msgs::msg::String>(
    "/planning/behavior_state", qos_rel,
    [this](const std_msgs::msg::String::SharedPtr msg) { behavior_state_ = msg->data; });

  pub_traj_ = create_publisher<av_msgs::msg::TrajectoryPointArray>("/planning/trajectory", qos_traj);

  behavior_state_ = "LANE_FOLLOWING";
  RCLCPP_INFO(get_logger(), "TrajectoryGenerator ready");
}

void TrajectoryGenerator::on_path(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) return;

  double speed = target_speed_;
  if (behavior_state_ == "STOPPING") speed = 0.0;
  if (behavior_state_ == "EMERGENCY_STOP") speed = 0.0;

  av_msgs::msg::TrajectoryPointArray traj;
  traj.header.stamp = now();
  traj.header.frame_id = "map";

  double t = 0.0;
  for (size_t i = 0; i + 1 < msg->poses.size(); ++i) {
    const auto & a = msg->poses[i].pose.position;
    const auto & b = msg->poses[i + 1].pose.position;
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double seg_len = std::sqrt(dx * dx + dy * dy);
    double heading = std::atan2(dy, dx);

    // Interpolate within segment at spacing_
    for (double s = 0.0; s < seg_len; s += spacing_) {
      av_msgs::msg::TrajectoryPoint pt;
      pt.header = traj.header;
      pt.pose.position.x = a.x + (dx / seg_len) * s;
      pt.pose.position.y = a.y + (dy / seg_len) * s;
      pt.pose.orientation.z = std::sin(heading / 2.0);
      pt.pose.orientation.w = std::cos(heading / 2.0);
      pt.longitudinal_velocity_mps = static_cast<float>(speed);
      pt.time_from_start = t + (s / std::max(speed, 0.01));
      traj.points.push_back(pt);

      if (traj.points.size() * spacing_ > horizon_) break;
    }
    t += seg_len / std::max(speed, 0.01);
    if (traj.points.size() * spacing_ > horizon_) break;
  }

  pub_traj_->publish(traj);
}

}  // namespace av_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_planning::TrajectoryGenerator)
