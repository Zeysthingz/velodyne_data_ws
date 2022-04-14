#ifndef PCLDATACAMP_ROSRELATED_H
#define PCLDATACAMP_ROSRELATED_H

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class RosRelated {
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;


  static void PublishCloud(const Cloud::ConstPtr &cloud_in,
//                           const rclcpp::Publisher &publisher);
//publisherları içine degısken olarak alan bır fonksıyon
                           const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher);

};

#endif //PCLDATACAMP_ROSRELATED_H
