#include "av_control/pid_speed_controller.hpp"

#include <av_msgs/msg/trajectory_point_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace av_control
{

PidSpeedController::PidSpeedController(const rclcpp::NodeOptions & options)
: Node("pid_speed_controller", options),
  integral_(0.0),
  prev_error_(0.0),
  prev_time_(now())
{
  declare_parameter("kp", 1.0);
  declare_parameter("ki", 0.1);
  declare_parameter("kd", 0.05);
  declare_parameter("max_acceleration", 3.0);
  declare_parameter("max_deceleration", 6.0);

  kp_ = get_parameter("kp").as_double();
  ki_ = get_parameter("ki").as_double();
  kd_ = get_parameter("kd").as_double();
  max_accel_ = get_parameter("max_acceleration").as_double();
  max_decel_ = get_parameter("max_deceleration").as_double();

  auto qos_traj = rclcpp::QoS(1).reliable().transient_local();
  auto qos_odom = rclcpp::QoS(10).reliable().volatile_durability();

  sub_traj_ = create_subscription<av_msgs::msg::TrajectoryPointArray>(
    "/planning/trajectory", qos_traj,
    [this](const av_msgs::msg::TrajectoryPointArray::SharedPtr msg) { trajectory_ = msg; });

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/odom", qos_odom,
    std::bind(&PidSpeedController::on_odom, this, std::placeholders::_1));

  pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/control/cmd", qos_odom);

  RCLCPP_INFO(get_logger(), "PidSpeedController ready (kp=%.2f ki=%.2f kd=%.2f)",
    kp_, ki_, kd_);
}

void PidSpeedController::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!trajectory_ || trajectory_->points.empty()) return;

  double current_speed = msg->twist.twist.linear.x;
  double target_speed = trajectory_->points.front().longitudinal_velocity_mps;

  rclcpp::Time t = now();
  double dt = (t - prev_time_).seconds();
  prev_time_ = t;
  if (dt <= 0.0 || dt > 1.0) return;

  double error = target_speed - current_speed;
  integral_ += error * dt;
  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  double accel = kp_ * error + ki_ * integral_ + kd_ * derivative;
  accel = std::clamp(accel, -max_decel_, max_accel_);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = current_speed + accel * dt;
  pub_cmd_->publish(cmd);
}

}  // namespace av_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_control::PidSpeedController)
