#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "edgellm_vlm_ros/vlm_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgellm_vlm_ros::VlmNode>());
  rclcpp::shutdown();
  return 0;
}
