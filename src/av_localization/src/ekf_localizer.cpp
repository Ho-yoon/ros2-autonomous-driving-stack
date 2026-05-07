#include "av_localization/ekf_localizer.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace av_localization
{

EkfLocalizer::EkfLocalizer(const rclcpp::NodeOptions & options)
: Node("ekf_localizer", options),
  tf_broadcaster_(this),
  state_(Eigen::VectorXd::Zero(6)),
  covariance_(Eigen::MatrixXd::Identity(6, 6) * 1.0)
{
  declare_parameter("process_noise_pos", 0.01);
  declare_parameter("process_noise_yaw", 0.001);
  declare_parameter("publish_rate", 50.0);

  process_noise_pos_ = get_parameter("process_noise_pos").as_double();
  process_noise_yaw_ = get_parameter("process_noise_yaw").as_double();

  auto qos_reliable = rclcpp::QoS(10).reliable().volatile_durability();
  auto qos_pose = rclcpp::QoS(1).reliable().transient_local();

  sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/gnss_pose",
    qos_reliable,
    std::bind(&EkfLocalizer::on_gnss_pose, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/sensing/imu",
    qos_reliable,
    std::bind(&EkfLocalizer::on_imu, this, std::placeholders::_1));

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose", qos_pose);

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(
    "/localization/odom", qos_reliable);

  last_predict_time_ = now();

  double rate = get_parameter("publish_rate").as_double();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&EkfLocalizer::publish, this));

  RCLCPP_INFO(get_logger(), "EkfLocalizer ready at %.0f Hz", rate);
}

void EkfLocalizer::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time t = msg->header.stamp;
  double dt = (t - last_predict_time_).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    last_predict_time_ = t;
    return;
  }
  last_predict_time_ = t;

  // State: [x, y, z, roll, pitch, yaw]
  // Predict step: integrate angular velocity for yaw
  double yaw_rate = msg->angular_velocity.z;
  state_(5) += yaw_rate * dt;

  // Propagate position using constant velocity assumption
  // (velocity would come from odometry in a full implementation)
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  state_(0) += 0.5 * ax * dt * dt;
  state_(1) += 0.5 * ay * dt * dt;

  // Inflate covariance
  covariance_(0, 0) += process_noise_pos_ * dt;
  covariance_(1, 1) += process_noise_pos_ * dt;
  covariance_(5, 5) += process_noise_yaw_ * dt;
}

void EkfLocalizer::on_gnss_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // EKF update step: measurement z = [x, y, yaw]
  Eigen::Vector3d z(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    tf2::getYaw(msg->pose.pose.orientation));

  Eigen::Vector3d h(state_(0), state_(1), state_(5));
  Eigen::Vector3d innov = z - h;

  // Wrap yaw innovation to [-pi, pi]
  while (innov(2) > M_PI) innov(2) -= 2 * M_PI;
  while (innov(2) < -M_PI) innov(2) += 2 * M_PI;

  // Mahalanobis gate (3-sigma)
  Eigen::Matrix3d S;
  S(0, 0) = covariance_(0, 0) + msg->pose.covariance[0];
  S(1, 1) = covariance_(1, 1) + msg->pose.covariance[7];
  S(2, 2) = covariance_(5, 5) + msg->pose.covariance[35];
  double d2 = innov.transpose() * S.inverse() * innov;
  if (d2 > 9.0) {
    RCLCPP_WARN(get_logger(), "GNSS rejected: Mahalanobis=%.2f", std::sqrt(d2));
    return;
  }

  // Kalman gain (simplified: only x, y, yaw are observed)
  state_(0) += (covariance_(0, 0) / S(0, 0)) * innov(0);
  state_(1) += (covariance_(1, 1) / S(1, 1)) * innov(1);
  state_(5) += (covariance_(5, 5) / S(2, 2)) * innov(2);

  covariance_(0, 0) *= (1.0 - covariance_(0, 0) / S(0, 0));
  covariance_(1, 1) *= (1.0 - covariance_(1, 1) / S(1, 1));
  covariance_(5, 5) *= (1.0 - covariance_(5, 5) / S(2, 2));
}

void EkfLocalizer::publish()
{
  auto stamp = now();

  // Publish pose
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = "map";
  pose_msg.pose.pose.position.x = state_(0);
  pose_msg.pose.pose.position.y = state_(1);
  pose_msg.pose.pose.position.z = state_(2);
  tf2::Quaternion q;
  q.setRPY(state_(3), state_(4), state_(5));
  pose_msg.pose.pose.orientation = tf2::toMsg(q);
  pose_msg.pose.covariance[0]  = covariance_(0, 0);
  pose_msg.pose.covariance[7]  = covariance_(1, 1);
  pose_msg.pose.covariance[35] = covariance_(5, 5);
  pub_pose_->publish(pose_msg);

  // Publish odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose = pose_msg.pose.pose;
  pub_odom_->publish(odom_msg);

  // Broadcast TF: map → odom → base_link
  geometry_msgs::msg::TransformStamped tf_map_odom;
  tf_map_odom.header.stamp = stamp;
  tf_map_odom.header.frame_id = "map";
  tf_map_odom.child_frame_id = "odom";
  tf_map_odom.transform.rotation.w = 1.0;
  tf_broadcaster_.sendTransform(tf_map_odom);

  geometry_msgs::msg::TransformStamped tf_odom_base;
  tf_odom_base.header.stamp = stamp;
  tf_odom_base.header.frame_id = "odom";
  tf_odom_base.child_frame_id = "base_link";
  tf_odom_base.transform.translation.x = state_(0);
  tf_odom_base.transform.translation.y = state_(1);
  tf_odom_base.transform.translation.z = state_(2);
  tf_odom_base.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_.sendTransform(tf_odom_base);
}

}  // namespace av_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_localization::EkfLocalizer)
