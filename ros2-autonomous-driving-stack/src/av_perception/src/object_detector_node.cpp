#include "av_perception/object_detector_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <av_msgs/msg/detected_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace av_perception
{

ObjectDetectorNode::ObjectDetectorNode(const rclcpp::NodeOptions & options)
: Node("object_detector", options), next_id_(0)
{
  declare_parameter("cluster_tolerance", 0.5);
  declare_parameter("min_cluster_size", 10);
  declare_parameter("max_cluster_size", 2000);

  cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
  min_cluster_size_ = get_parameter("min_cluster_size").as_int();
  max_cluster_size_ = get_parameter("max_cluster_size").as_int();

  auto qos_be = rclcpp::QoS(5).best_effort().volatile_durability();
  auto qos_rel = rclcpp::QoS(10).reliable().volatile_durability();

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar_points",
    qos_be,
    std::bind(&ObjectDetectorNode::on_cloud, this, std::placeholders::_1));

  pub_ = create_publisher<av_msgs::msg::DetectedObjectArray>("/perception/objects", qos_rel);

  RCLCPP_INFO(get_logger(), "ObjectDetectorNode ready");
}

void ObjectDetectorNode::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  // Euclidean cluster extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  av_msgs::msg::DetectedObjectArray detections;
  detections.header = msg->header;

  for (const auto & indices : cluster_indices) {
    float x_sum = 0, y_sum = 0, z_sum = 0;
    float x_min = 1e6, y_min = 1e6, z_min = 1e6;
    float x_max = -1e6, y_max = -1e6, z_max = -1e6;

    for (int idx : indices.indices) {
      const auto & pt = cloud->points[idx];
      x_sum += pt.x; y_sum += pt.y; z_sum += pt.z;
      x_min = std::min(x_min, pt.x); x_max = std::max(x_max, pt.x);
      y_min = std::min(y_min, pt.y); y_max = std::max(y_max, pt.y);
      z_min = std::min(z_min, pt.z); z_max = std::max(z_max, pt.z);
    }
    float n = static_cast<float>(indices.indices.size());

    av_msgs::msg::DetectedObject obj;
    obj.header = msg->header;
    obj.id = next_id_++;
    obj.label = "unknown";
    obj.confidence = 0.8f;
    obj.pose.pose.position.x = x_sum / n;
    obj.pose.pose.position.y = y_sum / n;
    obj.pose.pose.position.z = z_sum / n;
    obj.pose.pose.orientation.w = 1.0;
    obj.dimensions.x = x_max - x_min;
    obj.dimensions.y = y_max - y_min;
    obj.dimensions.z = z_max - z_min;

    detections.objects.push_back(obj);
  }

  pub_->publish(detections);
}

}  // namespace av_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(av_perception::ObjectDetectorNode)
