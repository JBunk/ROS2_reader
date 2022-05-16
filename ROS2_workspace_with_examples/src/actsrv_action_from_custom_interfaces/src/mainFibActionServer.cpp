#include "FibActionServer.hpp"
#include "rclcpp/rclcpp.hpp"  


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<FibActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}