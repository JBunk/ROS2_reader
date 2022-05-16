#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "SubscriberNode.h"

using std::placeholders::_1;

SubscriberNode::SubscriberNode(): Node("minimal_subscriber"){
  // initialize the subscription with:
  // "topic" name of the topic
  // 10: size of the queue buffer for backup
  // binding the function topic_callback()
  // with std::placeholders::_1 we place a placeholder for the 
  // function parameter (the message from the topic)
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&SubscriberNode::topic_callback, this, _1));
}


void SubscriberNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

