#pragma once

#include <av_msgs/msg/detected_object.hpp>
#include <av_msgs/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace av_perception
{

class ObjectDetectorNode : public rclcpp::Node
{
public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions & options);

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<av_msgs::msg::DetectedObjectArray>::SharedPtr pub_;

  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  uint32_t next_id_;
};

}  // namespace av_perception
