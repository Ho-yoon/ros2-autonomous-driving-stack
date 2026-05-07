#include "av_control/pure_pursuit_controller.hpp"

#include <cmath>

#include <av_msgs/msg/trajectory_point_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace av_control
{

PurePursuitController::PurePursuitController(const rclcpp::NodeOptions & options)
: Node("pure_pursuit_controller", options)
{
  declare_parameter("lookahead_distance", 5.0);
  declare_parameter("wheelbase", 2.7);
  declare_parameter("max_steering_angle", 0.6);
  declare_parameter("min_speed_for_stanley", 1.0);

  lookahead_ = get_parameter("lookahead_distance").as_double();
  wheelbase_ = get_parameter("wheelbase").as_double();
  max_steer_ = get_parameter("max_steering_angle").as_double();
  min_speed_stanley_ = get_parameter("min_speed_for_stanley").as_double();

  auto qos_traj = rclcpp::QoS(1).reliable().transient_local();
  auto qos_odom = rclcpp::QoS(10).reliable().volatile_durability();

  sub_traj_ = create_subscription<av_msgs::msg::TrajectoryPointArray>(
    "/planning/trajectory", qos_traj,
    [this](const av_msgs::msg::TrajectoryPointArray::SharedPtr msg) { trajectory_ = msg; });

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/odom", qos_odom,
    std::bind(&PurePursuitController::on_odom, this, std::placeholders::_1));

  pub_steering_ = create_publisher<std_msgs::msg::Float32>("/control/steering", qos_odom);
  pub_error_   = create_publisher<std_msgs::msg::Float32>("/control/lateral_error", qos_odom);

  RCLCPP_INFO(get_logger(), "PurePursuitController ready (lookahead=%.1f m)", lookahead_);
}

void PurePursuitController::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!trajectory_ || trajectory_->points.empty()) return;

  double ego_x = msg->pose.pose.position.x;
  double ego_y = msg->pose.pose.position.y;

  // Check trajectory staleness
  double traj_age = (now() - rclcpp::Time(trajectory_->header.stamp)).seconds();
  if (traj_age > 0.5) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Trajectory stale (%.2f s)", traj_age);
    return;
  }

  // Find lookahead point
  const av_msgs::msg::TrajectoryPoint * target = nullptr;
  for (const auto & pt : trajectory_->points) {
    double dx = pt.pose.position.x - ego_x;
    double dy = pt.pose.position.y - ego_y;
    if (std::sqrt(dx * dx + dy * dy) >= lookahead_) {
      target = &pt;
      break;
    }
  }
  if (!target) target = &trajectory_->points.back();

  // Pure Pursuit geometry
  double dx = target->pose.position.x - ego_x;
  double dy = target->pose.position.y - ego_y;
  double alpha = std::atan2(dy, dx) - yaw_from_quat(msg->pose.pose.orientation);
  double curvature = 2.0 * std::sin(alpha) / lookahead_;
  double steering = std::atan(wheelbase_ * curvature);
  steering = std::clamp(steering, -max_steer_, max_steer_);

  // Lateral error = perpendicular distance to nearest point
  double lat_error = std::sqrt(dx * dx + dy * dy) * std::sin(alpha);

  std_msgs::msg::Float32 steer_msg;
  steer_msg.data = static_cast<float>(steering);
  pub_steering_->publish(steer_msg);

  std_msgs::msg::Float32 err_msg;
  err_msg.data = static_cast<float>(lat_error);
  pub_error_->publish(err_msg);
}

double PurePursuitController::yaw_from_quat(
  const geometry_msgs::msg::Quaternion & q) const
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace av_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_control::PurePursuitController)
