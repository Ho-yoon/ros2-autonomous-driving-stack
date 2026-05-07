#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace av_sensing
{

class LidarPreprocessor : public rclcpp::Node
{
public:
  explicit LidarPreprocessor(const rclcpp::NodeOptions & options);

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  double voxel_leaf_size_;
  double min_range_;
  double max_range_;
};

}  // namespace av_sensing
