#include "av_localization/gnss_imu_fusion.hpp"

#include <GeographicLib/LocalCartesian.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace av_localization
{

GnssImuFusion::GnssImuFusion(const rclcpp::NodeOptions & options)
: Node("gnss_imu_fusion", options), origin_set_(false)
{
  declare_parameter("position_stddev", 1.5);   // metres
  declare_parameter("heading_stddev", 0.1);    // radians

  position_stddev_ = get_parameter("position_stddev").as_double();

  auto qos_reliable = rclcpp::QoS(5).reliable().volatile_durability();

  sub_gnss_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/sensing/gnss",
    qos_reliable,
    std::bind(&GnssImuFusion::on_gnss, this, std::placeholders::_1));

  pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/gnss_pose", qos_reliable);

  RCLCPP_INFO(get_logger(), "GnssImuFusion ready");
}

void GnssImuFusion::on_gnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status < 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No GNSS fix");
    return;
  }

  if (!origin_set_) {
    proj_.Reset(msg->latitude, msg->longitude, msg->altitude);
    origin_set_ = true;
    RCLCPP_INFO(get_logger(), "GNSS origin set: lat=%.6f lon=%.6f", msg->latitude, msg->longitude);
  }

  double x, y, z;
  proj_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = "map";
  out.pose.pose.position.x = x;
  out.pose.pose.position.y = y;
  out.pose.pose.position.z = z;
  out.pose.pose.orientation.w = 1.0;

  double var = position_stddev_ * position_stddev_;
  out.pose.covariance[0]  = var;
  out.pose.covariance[7]  = var;
  out.pose.covariance[14] = var;
  out.pose.covariance[35] = 0.01;  // yaw: not observable from GNSS alone

  pub_->publish(out);
}

}  // namespace av_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_localization::GnssImuFusion)
