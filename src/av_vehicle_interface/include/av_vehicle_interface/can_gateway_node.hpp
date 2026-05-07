#pragma once

#include <string>

#include <av_msgs/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace av_vehicle_interface
{

class CanGatewayNode : public rclcpp::Node
{
public:
  explicit CanGatewayNode(const rclcpp::NodeOptions & options);

private:
  void on_command(const av_msgs::msg::VehicleState::SharedPtr msg);
  void send_can_frame(int can_id, float value);

  rclcpp::Subscription<av_msgs::msg::VehicleState>::SharedPtr sub_;

  std::string can_iface_;
  int steer_id_;
  int throttle_id_;
};

}  // namespace av_vehicle_interface
