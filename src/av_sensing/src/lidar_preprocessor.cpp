#include "av_sensing/lidar_preprocessor.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace av_sensing
{

LidarPreprocessor::LidarPreprocessor(const rclcpp::NodeOptions & options)
: Node("lidar_preprocessor", options)
{
  declare_parameter("voxel_leaf_size", 0.1);
  declare_parameter("min_range", 0.5);
  declare_parameter("max_range", 100.0);
  declare_parameter("input_topic", "/sensing/lidar_raw");
  declare_parameter("output_topic", "/sensing/lidar_points");

  voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
  min_range_ = get_parameter("min_range").as_double();
  max_range_ = get_parameter("max_range").as_double();

  auto qos_best_effort = rclcpp::QoS(5).best_effort().volatile_durability();

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    get_parameter("input_topic").as_string(),
    qos_best_effort,
    std::bind(&LidarPreprocessor::on_cloud, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    get_parameter("output_topic").as_string(), qos_best_effort);

  RCLCPP_INFO(get_logger(), "LidarPreprocessor ready (voxel=%.2f m)", voxel_leaf_size_);
}

void LidarPreprocessor::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  // Range filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  filtered->reserve(cloud->size());
  for (const auto & pt : *cloud) {
    float r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (r >= min_range_ && r <= max_range_) {
      filtered->push_back(pt);
    }
  }

  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(filtered);
  vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  vg.filter(*downsampled);

  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*downsampled, out);
  out.header = msg->header;
  pub_->publish(out);
}

}  // namespace av_sensing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_sensing::LidarPreprocessor)
