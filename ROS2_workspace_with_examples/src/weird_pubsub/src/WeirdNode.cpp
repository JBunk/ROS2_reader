#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "WeirdNode.h"

using namespace std::chrono_literals; // allows us to use `ms'.
using std::placeholders::_1; 

WeirdNode::WeirdNode(const std::string & name): Node(name){
    // initializing the publisher using the create_publisher function of the parent:
    // "topic" the name of the topic
    // 10: the size of the queue
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // initializing the timer and binding it with the function timer_callback():
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&WeirdNode::topic_callback, this, _1));

    std::string content = "Wat heb je?";
    publishMessage(content);
}

void WeirdNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    std::string content = msg->data.c_str();
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", content.c_str());

    if(content.size() > 1){
        // copy string from msg, expect last character:
        std::string new_content = content.substr(0, content.size()-1);
    
        for(int i=0; i<1000000000; i++){} // wait a bit.

        publishMessage(new_content);
    }
}

void WeirdNode::publishMessage(std::string content) const{
    // a new message:
    auto message = std_msgs::msg::String();
    // adding data 
    message.data = content;
    // publishing the message on the topic:
    publisher_->publish(message);
}



