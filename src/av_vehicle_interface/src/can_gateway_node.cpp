#include "av_vehicle_interface/can_gateway_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace av_vehicle_interface
{

CanGatewayNode::CanGatewayNode(const rclcpp::NodeOptions & options)
: Node("can_gateway", options)
{
  declare_parameter("can_interface", "can0");
  declare_parameter("steering_can_id", 0x18);
  declare_parameter("throttle_can_id", 0x19);

  can_iface_ = get_parameter("can_interface").as_string();
  steer_id_  = get_parameter("steering_can_id").as_int();
  throttle_id_ = get_parameter("throttle_can_id").as_int();

  auto qos_cmd = rclcpp::QoS(1).reliable().volatile_durability();

  sub_ = create_subscription<av_msgs::msg::VehicleState>(
    "/vehicle/command", qos_cmd,
    std::bind(&CanGatewayNode::on_command, this, std::placeholders::_1));

  // In a real deployment: open SocketCAN socket here
  // socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  RCLCPP_INFO(get_logger(), "CanGatewayNode ready on %s", can_iface_.c_str());
}

void CanGatewayNode::on_command(const av_msgs::msg::VehicleState::SharedPtr msg)
{
  if (msg->emergency_stop) {
    RCLCPP_ERROR(get_logger(), "EMERGENCY_STOP received — broadcasting zero-motion frame");
    send_can_frame(steer_id_,   0.0f);
    send_can_frame(throttle_id_, 0.0f);
    return;
  }

  send_can_frame(steer_id_,    msg->steering_tire_angle);
  send_can_frame(throttle_id_, msg->longitudinal_velocity);
}

void CanGatewayNode::send_can_frame(int can_id, float value)
{
  // Placeholder: real implementation would write a struct can_frame to socket_fd_
  RCLCPP_DEBUG(get_logger(), "CAN 0x%02X → %.3f", can_id, value);
  (void)can_id;
  (void)value;
}

}  // namespace av_vehicle_interface

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_vehicle_interface::CanGatewayNode)
