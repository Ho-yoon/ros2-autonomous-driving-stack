#pragma once

#include <av_msgs/msg/vehicle_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace av_vehicle_interface
{

class VehicleCommandAdapter : public rclcpp::Node
{
public:
  explicit VehicleCommandAdapter(const rclcpp::NodeOptions & options);

private:
  void on_steering(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steering_;
  rclcpp::Publisher<av_msgs::msg::VehicleState>::SharedPtr pub_;

  geometry_msgs::msg::Twist::SharedPtr last_cmd_;
  double max_steer_;
  double max_speed_;
};

}  // namespace av_vehicle_interface
