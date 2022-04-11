#ifndef PCL_UDACITY_TUT_H
#define PCL_UDACITY_TUT_H
#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PclStuff.h"
#include "RosRelated.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <boost/smart_ptr.hpp>
#include <pcl/common/common.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class UdacityTut : public rclcpp::Node {
 public:
//  UdacityTut();
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;


  explicit UdacityTut();
// private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_downsampled_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_groundless_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_eucledian_clusterer_;
//  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_velodyne_points_;
//  std::function<void(std::shared_ptr < sensor_msgs::msg::PointCloud2 > )> functionSub;


// fonksıyonu void yapar

  void CallbackLaser(const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud) const;
};

#endif //PCL_UDACITY_TUT_H
