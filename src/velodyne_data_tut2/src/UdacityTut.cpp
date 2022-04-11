#include <velodyne_data_tut2/UdacityTut.h>
//#include "sensor_msgs/msg/point_cloud2.hpp"
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl/pcl_config.h>
using namespace std::chrono_literals;
//TODO OLD CODE

//UdacityTut::UdacityTut public rclcpp::Node {
UdacityTut::UdacityTut()
    : Node("UdacityTut") {

  // Publishers
  //// Point Clouds

  std::cout << PCL_VERSION << std::endl;
  std::cout<< "hello5\n";
  pub_cloud_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_raw",
      10);
  std::cout<< "hello6\n";
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
      "/velodyne_points", 10, callback);
}


//
//  sub_velodyne_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//      "/right/velodyne_points", 10, std::bind(&UdacityTut::CallbackLaser, this, _1));

//  functionSub = std::bind(&UdacityTut::CallbackLaser,
//                          this, std::placeholders::_1);
//  sub_velodyne_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//      "/mid_r/velodyne_points", 10, functionSub);
//  std::cout << sub_velodyne_points_ << std::endl;



//private:
//  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
void UdacityTut::CallbackLaser(const sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud) const {
  std::cout<< "hello1\n";

  auto lastIndex = msg_cloud->data.size() - 2;
  std::cout<< "hello2\n";

  auto pcSize = std::to_string(msg_cloud->data[lastIndex]).c_str();
  std::cout<< "hello3\n";

  std::cout<< "hello4\n";

  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", pcSize);

    // Convert Point Cloud Message to PCL Point Cloud format
  Cloud::Ptr cloud_in(new Cloud);
  pcl::fromROSMsg(*msg_cloud, *cloud_in);
  RosRelated::PublishCloud(cloud_in, pub_cloud_raw_);
  std::cout<<"hello world"<<std::endl;

  //  // Downsample it
  Cloud::Ptr cloud_ds = PclStuff::Downsample(cloud_in, 0.2f);
  RosRelated::PublishCloud(cloud_ds, pub_cloud_downsampled_);
////
////   Remove ground from the downsampled point cloud
////  downsample yanı arttırılmıs data ground remover fonksıyonuna gırıyor
  Cloud::Ptr cloud_groundless = PclStuff::GroundRemover(cloud_ds, 0.3f);
  RosRelated::PublishCloud(cloud_groundless, pub_cloud_groundless_);

  ////   My addition cpublisdshes a node named as cloud_ground
////   gives downsample object cloud_ds to ground finder function
  Cloud::Ptr cloud_ground = PclStuff::GroundFinder(cloud_ds, 0.5f);
  RosRelated::PublishCloud(cloud_ground, pub_cloud_ground_);
////
////
//// Euclidean Cluster Function
  Cloud::Ptr cloud_cluster = PclStuff::EucledianCluster(cloud_groundless);
  RosRelated::PublishCloud(cloud_cluster, pub_eucledian_clusterer_);
//
//}

}

int main(int argc, char *argv[]) {
  std::cout<< "deneme";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdacityTut>());
  rclcpp::shutdown();
  return 0;
}

// todo old
//void
//UdacityTut::CallbackLaser(const sensor_msgs::msg::PointCloud2::SharedPtr &msg_cloud) {
//  // Convert Point Cloud Message to PCL Point Cloud format
//  Cloud::Ptr cloud_in(new Cloud);
//  pcl::fromROSMsg(*msg_cloud, *cloud_in);
//  RosRelated::PublishCloud(cloud_in, pub_cloud_raw_);
//  std::cout<<"hello worl"<<std::endl;
//  // Downsample it
//  Cloud::Ptr cloud_ds = PclStuff::Downsample(cloud_in, 0.2f);
//  RosRelated::PublishCloud(cloud_ds, pub_cloud_downsampled_);
////
////   Remove ground from the downsampled point cloud
////  downsample yanı arttırılmıs data ground remover fonksıyonuna gırıyor
//  Cloud::Ptr cloud_groundless = PclStuff::GroundRemover(cloud_ds, 0.3f);
//  RosRelated::PublishCloud(cloud_groundless, pub_cloud_groundless_);
////
////   My addition cpublisdshes a node named as cloud_ground
////   gives downsample object cloud_ds to ground finder function
//  Cloud::Ptr cloud_ground = PclStuff::GroundFinder(cloud_ds, 0.5f);
//  RosRelated::PublishCloud(cloud_ground, pub_cloud_ground_);
////
////
//// Euclidean Cluster Function
//  Cloud::Ptr cloud_cluster = PclStuff::EucledianCluster(cloud_groundless);
//  RosRelated::PublishCloud(cloud_cluster, pub_eucledian_clusterer_);
//
//}

//int main(int argc, char *argv[]) {
//  rclcpp::init(argc, argv);
//  auto node = std::make_shared<rclcpp::Node>("UdacityTut");
////  std::cout<<node->get_node_names().data()<< std::endl;
////  std::cout<<node->get_node_names().data()<< std::endl;
//  rclcpp::spin(node);
//  rclcpp::shutdown();
//  return 0;
//}
