#include "rclcpp/rclcpp.hpp"
#include <ros/spinner.h>
#include "UdacityTut.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "udacity_pcl_tut_node");
  ros::NodeHandle nh;
  UdacityTut udacity_tut(nh);

  rclcpp::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
