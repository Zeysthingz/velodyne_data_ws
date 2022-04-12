#ifndef PCL_UDACITY_TUT_H
#define PCL_UDACITY_TUT_H
#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PclStuff.h"
#include "RosRelated.h"
#include "iostream"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace UdacityTutNO {
class UdacityTut : public rclcpp::Node {
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  explicit UdacityTut(const rclcpp::NodeOptions &node_options);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_downsampled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_groundless_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_eucledian_clusterer_;

  void CallbackLaser(const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud);
};
}  // namespace UdacityTut.

#endif //PCL_UDACITY_TUT_H
