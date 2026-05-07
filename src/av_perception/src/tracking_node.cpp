#include "av_perception/tracking_node.hpp"

#include <av_msgs/msg/tracked_object.hpp>

#include <rclcpp/rclcpp.hpp>

namespace av_perception
{

TrackingNode::TrackingNode(const rclcpp::NodeOptions & options)
: Node("tracking", options), next_track_id_(0)
{
  declare_parameter("max_missed_frames", 5);
  declare_parameter("association_distance", 3.0);

  max_missed_ = get_parameter("max_missed_frames").as_int();
  assoc_dist_ = get_parameter("association_distance").as_double();

  auto qos_rel = rclcpp::QoS(10).reliable().volatile_durability();

  sub_ = create_subscription<av_msgs::msg::DetectedObjectArray>(
    "/perception/objects",
    qos_rel,
    std::bind(&TrackingNode::on_detections, this, std::placeholders::_1));

  pub_ = create_publisher<av_msgs::msg::TrackedObjectArray>("/perception/tracked_objects", qos_rel);

  RCLCPP_INFO(get_logger(), "TrackingNode ready");
}

void TrackingNode::on_detections(const av_msgs::msg::DetectedObjectArray::SharedPtr msg)
{
  // Greedy nearest-neighbour association (SORT-inspired)
  std::vector<bool> matched(msg->objects.size(), false);

  for (auto & track : tracks_) {
    track.missed++;
    double best_dist = assoc_dist_;
    int best_idx = -1;

    for (size_t i = 0; i < msg->objects.size(); ++i) {
      if (matched[i]) continue;
      const auto & det = msg->objects[i];
      double dx = det.pose.pose.position.x - track.detection.pose.pose.position.x;
      double dy = det.pose.pose.position.y - track.detection.pose.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = static_cast<int>(i);
      }
    }

    if (best_idx >= 0) {
      track.detection = msg->objects[best_idx];
      track.missed = 0;
      track.age++;
      matched[best_idx] = true;
    }
  }

  // Spawn new tracks for unmatched detections
  for (size_t i = 0; i < msg->objects.size(); ++i) {
    if (!matched[i]) {
      Track t;
      t.track_id = next_track_id_++;
      t.detection = msg->objects[i];
      t.age = 1;
      t.missed = 0;
      tracks_.push_back(t);
    }
  }

  // Prune stale tracks
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const Track & t) { return t.missed > max_missed_; }),
    tracks_.end());

  // Publish
  av_msgs::msg::TrackedObjectArray out;
  out.header = msg->header;
  for (const auto & track : tracks_) {
    av_msgs::msg::TrackedObject obj;
    obj.detection = track.detection;
    obj.track_id = track.track_id;
    obj.age = track.age;
    obj.existence_probability = std::min(1.0f, track.age / 5.0f);
    out.objects.push_back(obj);
  }
  pub_->publish(out);
}

}  // namespace av_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_perception::TrackingNode)
