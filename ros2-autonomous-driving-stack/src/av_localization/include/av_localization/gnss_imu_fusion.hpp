#pragma once

#include <GeographicLib/LocalCartesian.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace av_localization
{

class GnssImuFusion : public rclcpp::Node
{
public:
  explicit GnssImuFusion(const rclcpp::NodeOptions & options);

private:
  void on_gnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  GeographicLib::LocalCartesian proj_;
  bool origin_set_;
  double position_stddev_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

}  // namespace av_localization
