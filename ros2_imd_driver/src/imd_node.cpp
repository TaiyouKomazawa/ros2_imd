#include <rclcpp/rclcpp.hpp>
#include "imd_ros.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMDNode>());
  rclcpp::shutdown();
  return 0;
}
