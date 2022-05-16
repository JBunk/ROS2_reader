#include "rclcpp/rclcpp.hpp"
#include "FibActionClient.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<FibActionClient>();

  // Let the client send one goal and then close it:
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}