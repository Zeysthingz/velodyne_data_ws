#include "velodyne_data_tut2/RosRelated.h"

void RosRelated::PublishCloud(const Cloud::ConstPtr &cloud_in,
//                              const ros::Publisher &publisher) {
                             const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher)
{
  sensor_msgs::msg::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud_in, msg_cloud);
//  msg_cloud.header.stamp = ros::Time::now();
//şu ankı zamanı verıyor
  msg_cloud.header.stamp = rclcpp::Clock().now();
  msg_cloud.header.frame_id = "lidar_right";
//  class ok
  publisher->publish(msg_cloud);
}