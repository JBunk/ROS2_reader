#include "rclcpp/rclcpp.hpp"
#include "ServiceNode.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceNode>());
  rclcpp::shutdown();
  return 0;
}
