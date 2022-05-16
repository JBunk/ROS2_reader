#include "rclcpp/rclcpp.hpp"
// srv/AddThreeInts.srv becomes srv/add_three_ints.hpp:
#include "srvcli_custom_srv_in_pkg/srv/add_three_ints.hpp"
#include "ServiceNode.h"

// placeholders for the arguments
using std::placeholders::_1;
using std::placeholders::_2;

ServiceNode::ServiceNode(): Node("ServiceNode"){
    // initialise the service
    // <srv-message-type> in this case custom_interfaces::srv::AddTwoInts
    // "add_three_ints" the name of the topic
    // and a pointer to the function to be called when a service message is received.
    service_ = this->create_service<srvcli_custom_srv_in_pkg::srv::AddThreeInts>(
            "add_three_ints", 
            std::bind(&ServiceNode::add, this, _1, _2));
}

void ServiceNode::add(const std::shared_ptr<srvcli_custom_srv_in_pkg::srv::AddThreeInts::Request> request,
          std::shared_ptr<srvcli_custom_srv_in_pkg::srv::AddThreeInts::Response> response)
{
  // processing the request (calculating the sum) and assigning it to the response:
  response->sum = request->a + request->b + request->c;

  // for example purposes, printing to the logging:
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" "c: %ld",
                request->a, request->b, request->c);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", 
                (long int)response->sum);
}