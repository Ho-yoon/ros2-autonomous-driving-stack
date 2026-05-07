#include "av_vehicle_interface/vehicle_command_adapter.hpp"

#include <algorithm>
#include <cmath>

#include <av_msgs/msg/vehicle_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace av_vehicle_interface
{

VehicleCommandAdapter::VehicleCommandAdapter(const rclcpp::NodeOptions & options)
: Node("vehicle_command_adapter", options)
{
  declare_parameter("max_steering_angle", 0.6);
  declare_parameter("max_speed", 15.0);

  max_steer_ = get_parameter("max_steering_angle").as_double();
  max_speed_ = get_parameter("max_speed").as_double();

  auto qos_rel = rclcpp::QoS(5).reliable().volatile_durability();
  auto qos_cmd = rclcpp::QoS(1).reliable().volatile_durability();

  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
    "/control/cmd", qos_rel,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { last_cmd_ = msg; });

  sub_steering_ = create_subscription<std_msgs::msg::Float32>(
    "/control/steering", qos_rel,
    std::bind(&VehicleCommandAdapter::on_steering, this, std::placeholders::_1));

  pub_ = create_publisher<av_msgs::msg::VehicleState>("/vehicle/command", qos_cmd);

  RCLCPP_INFO(get_logger(), "VehicleCommandAdapter ready");
}

void VehicleCommandAdapter::on_steering(const std_msgs::msg::Float32::SharedPtr msg)
{
  av_msgs::msg::VehicleState cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";

  double steer = std::clamp(static_cast<double>(msg->data), -max_steer_, max_steer_);
  cmd.steering_tire_angle = static_cast<float>(steer);

  if (last_cmd_) {
    double speed = std::clamp(last_cmd_->linear.x, 0.0, max_speed_);
    cmd.longitudinal_velocity = static_cast<float>(speed);
    cmd.gear_shift = speed > 0.0 ? 3.0f : 2.0f;  // D or N
  }

  cmd.emergency_stop = false;
  pub_->publish(cmd);
}

}  // namespace av_vehicle_interface

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_vehicle_interface::VehicleCommandAdapter)
