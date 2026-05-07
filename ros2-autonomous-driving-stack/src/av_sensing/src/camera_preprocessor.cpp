#include "av_sensing/camera_preprocessor.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av_sensing
{

CameraPreprocessor::CameraPreprocessor(const rclcpp::NodeOptions & options)
: Node("camera_preprocessor", options)
{
  declare_parameter("fx", 900.0);
  declare_parameter("fy", 900.0);
  declare_parameter("cx", 640.0);
  declare_parameter("cy", 360.0);
  declare_parameter("input_topic", "/sensing/camera_raw");
  declare_parameter("output_topic", "/sensing/camera_image");

  double fx = get_parameter("fx").as_double();
  double fy = get_parameter("fy").as_double();
  double cx = get_parameter("cx").as_double();
  double cy = get_parameter("cy").as_double();

  // clang-format off
  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
    fx,  0, cx,
     0, fy, cy,
     0,  0,  1);
  // clang-format on
  dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

  auto qos = rclcpp::QoS(3).best_effort().volatile_durability();

  sub_ = create_subscription<sensor_msgs::msg::Image>(
    get_parameter("input_topic").as_string(),
    qos,
    std::bind(&CameraPreprocessor::on_image, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::Image>(
    get_parameter("output_topic").as_string(), qos);

  RCLCPP_INFO(get_logger(), "CameraPreprocessor ready");
}

void CameraPreprocessor::on_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
    return;
  }

  cv::Mat undistorted;
  cv::undistort(cv_ptr->image, undistorted, camera_matrix_, dist_coeffs_);
  cv_ptr->image = undistorted;

  pub_->publish(*cv_ptr->toImageMsg());
}

}  // namespace av_sensing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_sensing::CameraPreprocessor)
