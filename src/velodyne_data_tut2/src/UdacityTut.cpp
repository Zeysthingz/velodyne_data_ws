#include <velodyne_data_tut2/UdacityTut.h>

using namespace std::chrono_literals;

namespace UdacityTutNO {

UdacityTut::UdacityTut(const rclcpp::NodeOptions &node_options)
    : Node("point_cloud_node", node_options) {

  // Publishers
  //// Point Clouds

  std::cout << PCL_VERSION << std::endl;
  std::cout << PCL_VERSION << std::endl;
  std::cout << PCL_VERSION << std::endl;
  std::cout << PCL_VERSION << std::endl;
  std::cout << PCL_VERSION << std::endl;
  std::cout << PCL_VERSION << std::endl;
  std::cout << PCL_VERSION << std::endl;
  std::cout << PCL_VERSION << std::endl;

  pub_cloud_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_raw",
      10);
  pub_cloud_downsampled_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_downsampled",
      10);
  pub_cloud_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_ground",
      10);
  pub_cloud_groundless_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_groundless",
      10);
  pub_eucledian_clusterer_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_cluster",
      10);

  auto callback = [this](sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud) {
    this->CallbackLaser(msg_cloud);
  };
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/right/velodyne_points", 10, callback);

}

void UdacityTut::CallbackLaser(const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud) {

  auto lastIndex = msg_cloud->data.size() - 2;

  auto pcSize = std::to_string(msg_cloud->data[lastIndex]).c_str();

  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", pcSize);

  // Convert Point Cloud Message to PCL Point Cloud format
  Cloud::Ptr cloud_in(new Cloud);
  pcl::fromROSMsg(*msg_cloud, *cloud_in);
  RosRelated::PublishCloud(cloud_in, pub_cloud_raw_);

  // Downsample it
  Cloud::Ptr cloud_ds = PclStuff::Downsample(cloud_in, 0.2f);
  RosRelated::PublishCloud(cloud_ds, pub_cloud_downsampled_);

////   Remove ground from the downsampled point cloud
////  downsample yanı arttırılmıs data ground remover fonksıyonuna gırıyor
  Cloud::Ptr cloud_groundless = PclStuff::GroundRemover(cloud_ds, 0.3f);
  RosRelated::PublishCloud(cloud_groundless, pub_cloud_groundless_);

  ////   My addition cpublisdshes a node named as cloud_ground
////   gives downsample object cloud_ds to ground finder function
  Cloud::Ptr cloud_ground = PclStuff::GroundFinder(cloud_ds, 0.5f);
  RosRelated::PublishCloud(cloud_ground, pub_cloud_ground_);

//// Euclidean Cluster Function
  Cloud::Ptr cloud_cluster = PclStuff::EucledianCluster(cloud_groundless);
  RosRelated::PublishCloud(cloud_cluster, pub_eucledian_clusterer_);
}
}
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
    UdacityTutNO::UdacityTut)




