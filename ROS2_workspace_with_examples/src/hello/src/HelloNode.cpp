#include "rclcpp/rclcpp.hpp"
#include "HelloNode.h"


HelloNode::HelloNode() : Node("HelloNode"){
    //initialisation of the timer:
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&HelloNode::timerCallback, this));
}
    
void HelloNode::timerCallback(){
    // print "Hello World!" in the logger:
    RCLCPP_INFO(this->get_logger(), "Hello World!");
}

