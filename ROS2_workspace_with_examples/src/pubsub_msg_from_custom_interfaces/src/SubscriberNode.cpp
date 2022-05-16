#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// the msg-type from the custom_interfaces packages.
// AddressBook.msg becomes address_book.hpp
#include "custom_interfaces/msg/address_book.hpp" 
#include "SubscriberNode.h"

using std::placeholders::_1;

SubscriberNode::SubscriberNode(): Node("minimal_subscriber"){
  // initialize the subscription with:
  // <message-type> in this case custom_interfaces::msg::AddressBook
  // "topic" name of the topic
  // 10: size of the queue buffer for backup
  // binding the function topic_callback()
  subscription_ = this->create_subscription<custom_interfaces::msg::AddressBook>(
    "topic", 10, std::bind(&SubscriberNode::topic_callback, this, _1));
}


void SubscriberNode::topic_callback(const custom_interfaces::msg::AddressBook::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s' he is '%i' messages old.", msg->first_name.c_str(), msg->age);
}

