#pragma once

#include <string>

#include <av_msgs/msg/tracked_object_array.hpp>
#include <av_msgs/srv/reset_scenario.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace av_planning
{

enum class State { LANE_FOLLOWING, STOPPING, EMERGENCY_STOP };

class BehaviorPlanner : public rclcpp::Node
{
public:
  explicit BehaviorPlanner(const rclcpp::NodeOptions & options);

private:
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void on_objects(const av_msgs::msg::TrackedObjectArray::SharedPtr msg);
  void transition(State next);
  std::string state_name(State s) const;
  void on_reset(
    const av_msgs::srv::ResetScenario::Request::SharedPtr req,
    av_msgs::srv::ResetScenario::Response::SharedPtr res);

  State state_;
  geometry_msgs::msg::Pose current_pose_;
  double stop_distance_;
  double ttc_threshold_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<av_msgs::msg::TrackedObjectArray>::SharedPtr sub_objects_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Service<av_msgs::srv::ResetScenario>::SharedPtr srv_reset_;
};

}  // namespace av_planning
