#include "octomap_gene/OctomaoGen.hpp"



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapNode>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "octomap node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}