#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_three_ints.hpp"
#include "ClientNode.h"

#include <chrono>

using namespace std::chrono_literals;
// placeholder for the arguments
using std::placeholders::_1;

ClientNode::ClientNode(): Node("ClientNode"){
    // initialise the client.
    // "add_three_ints" the name of the service
    client_ = this->create_client<custom_interfaces::srv::AddThreeInts>("add_three_ints");

    // call the service:
    makeRequest();
}

void ClientNode::makeRequest(){
    // make a request of the right srv-type:
    auto request = std::make_shared<custom_interfaces::srv::AddThreeInts::Request>();
    // add the data to the request
    request->a = 5;
    request->b = 7;
    request->c = 9;

    // wait for the service to exist.
    // if the service is never started this is a endless loop
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // send the request and bind the function call the handels the response
    auto future = client_->async_send_request(request, std::bind(&ClientNode::handleResponse, this, _1));
}

void ClientNode::handleResponse(rclcpp::Client<custom_interfaces::srv::AddThreeInts>::SharedFuture future){
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %i", result->sum);
}
