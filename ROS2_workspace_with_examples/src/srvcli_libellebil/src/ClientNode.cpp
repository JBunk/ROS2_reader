#include "rclcpp/rclcpp.hpp"
#include "ClientNode.h"

// needed for the use of the 's' notation in wait_for_service()
using namespace std::chrono_literals;

// placeholder for the arguments
using std::placeholders::_1;

ClientNode::ClientNode(): Node("ClientNode"){
    // initialise the client.
    // "word" the name of the service
    client_ = this->create_client<srvcli_libellebil::srv::Word>("words");

    // call the service:
    makeRequest("pollepel");
    makeRequest("lepel");
    makeRequest("halloween");

}

void ClientNode::makeRequest(const std::string & input){
    // make a request of the right srv-type:
    auto request = std::make_shared<srvcli_libellebil::srv::Word::Request>();
    // add the data to the request
    request->input_word = input;

    // wait for the service to exist.
    // if the service is never started this is a endless loop
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // send the request and bind the function call the handels the response
    auto future = client_->async_send_request(request, std::bind(&ClientNode::handleResponse, this, _1));
}

void ClientNode::handleResponse(rclcpp::Client<srvcli_libellebil::srv::Word>::SharedFuture future){
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Is it? %d . result? %s", result->is_it, result->output_word.c_str());
}
