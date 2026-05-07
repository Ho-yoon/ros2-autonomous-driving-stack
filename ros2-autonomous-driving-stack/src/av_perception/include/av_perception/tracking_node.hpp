#pragma once

#include <av_msgs/msg/detected_object.hpp>
#include <av_msgs/msg/detected_object_array.hpp>
#include <av_msgs/msg/tracked_object.hpp>
#include <av_msgs/msg/tracked_object_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace av_perception
{

struct Track
{
  uint32_t track_id;
  av_msgs::msg::DetectedObject detection;
  uint32_t age;
  int missed;
};

class TrackingNode : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options);

private:
  void on_detections(const av_msgs::msg::DetectedObjectArray::SharedPtr msg);

  rclcpp::Subscription<av_msgs::msg::DetectedObjectArray>::SharedPtr sub_;
  rclcpp::Publisher<av_msgs::msg::TrackedObjectArray>::SharedPtr pub_;

  std::vector<Track> tracks_;
  uint32_t next_track_id_;
  int max_missed_;
  double assoc_dist_;
};

}  // namespace av_perception
