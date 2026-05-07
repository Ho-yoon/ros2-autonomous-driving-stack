#pragma once

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av_sensing
{

class CameraPreprocessor : public rclcpp::Node
{
public:
  explicit CameraPreprocessor(const rclcpp::NodeOptions & options);

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

}  // namespace av_sensing
