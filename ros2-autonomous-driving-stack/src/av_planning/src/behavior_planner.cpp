#include "av_planning/behavior_planner.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace av_planning
{

BehaviorPlanner::BehaviorPlanner(const rclcpp::NodeOptions & options)
: Node("behavior_planner", options), state_(State::LANE_FOLLOWING)
{
  declare_parameter("stop_distance", 5.0);
  declare_parameter("obstacle_ttc_threshold", 3.0);  // seconds

  stop_distance_ = get_parameter("stop_distance").as_double();
  ttc_threshold_ = get_parameter("obstacle_ttc_threshold").as_double();

  auto qos_pose = rclcpp::QoS(1).reliable().transient_local();
  auto qos_rel  = rclcpp::QoS(10).reliable().volatile_durability();

  sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose", qos_pose,
    std::bind(&BehaviorPlanner::on_pose, this, std::placeholders::_1));

  sub_objects_ = create_subscription<av_msgs::msg::TrackedObjectArray>(
    "/perception/tracked_objects", qos_rel,
    std::bind(&BehaviorPlanner::on_objects, this, std::placeholders::_1));

  pub_state_ = create_publisher<std_msgs::msg::String>("/planning/behavior_state", qos_rel);

  srv_reset_ = create_service<av_msgs::srv::ResetScenario>(
    "/scenario/reset",
    std::bind(&BehaviorPlanner::on_reset, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "BehaviorPlanner ready");
}

void BehaviorPlanner::on_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
}

void BehaviorPlanner::on_objects(
  const av_msgs::msg::TrackedObjectArray::SharedPtr msg)
{
  // Check time since last message for staleness
  double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
  if (age > 0.3) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Tracked objects stale (%.2f s) — holding last known", age);
  }

  State next_state = State::LANE_FOLLOWING;

  for (const auto & obj : msg->objects) {
    double dx = obj.detection.pose.pose.position.x - current_pose_.position.x;
    double dy = obj.detection.pose.pose.position.y - current_pose_.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < stop_distance_) {
      next_state = State::STOPPING;
    }

    double rel_vx = obj.detection.velocity.linear.x;
    if (rel_vx < 0 && dist / (-rel_vx) < ttc_threshold_) {
      next_state = State::EMERGENCY_STOP;
      break;
    }
  }

  transition(next_state);
}

void BehaviorPlanner::transition(State next)
{
  if (next == state_) return;
  RCLCPP_INFO(get_logger(), "Behavior: %s -> %s",
    state_name(state_).c_str(), state_name(next).c_str());
  state_ = next;

  std_msgs::msg::String msg;
  msg.data = state_name(state_);
  pub_state_->publish(msg);
}

std::string BehaviorPlanner::state_name(State s) const
{
  switch (s) {
    case State::LANE_FOLLOWING: return "LANE_FOLLOWING";
    case State::STOPPING:       return "STOPPING";
    case State::EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

void BehaviorPlanner::on_reset(
  const av_msgs::srv::ResetScenario::Request::SharedPtr req,
  av_msgs::srv::ResetScenario::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Resetting for scenario: %s", req->scenario_id.c_str());
  state_ = State::LANE_FOLLOWING;
  res->success = true;
  res->message = "Reset to LANE_FOLLOWING";
}

}  // namespace av_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_planning::BehaviorPlanner)
