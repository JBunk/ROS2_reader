#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// msg/AddressBook.msg becomes msg/address_book.hpp:
#include "pubsub_custom_msg_in_pkg/msg/address_book.hpp"
#include "SubscriberNode.h"

using std::placeholders::_1;


SubscriberNode::SubscriberNode(): Node("minimal_subscriber"){
  // initialize the subscription with:
  // "topic" name of the topic
  // 10: size of the queue buffer for backup
  // binding the function topic_callback()
  subscription_ = this->create_subscription<pubsub_custom_msg_in_pkg::msg::AddressBook>(
    "topic", 10, std::bind(&SubscriberNode::topic_callback, this, _1));
}


void SubscriberNode::topic_callback(const pubsub_custom_msg_in_pkg::msg::AddressBook::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s' he is '%i' messages old.", msg->first_name.c_str(), msg->age);
}

