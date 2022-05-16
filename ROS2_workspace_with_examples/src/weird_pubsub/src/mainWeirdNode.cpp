#include "rclcpp/rclcpp.hpp"
#include "WeirdNode.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeirdNode>("Weirdo"));
  rclcpp::shutdown();
  return 0;
}
