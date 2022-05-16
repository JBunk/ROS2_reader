#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// msg/AddressBook.msg becomes msg/address_book.hpp:
#include "pubsub_custom_msg_in_pkg/msg/address_book.hpp"
#include "PublisherNode.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


PublisherNode::PublisherNode(): Node("PublisherNode"), count_(0){
  // initializing the publisher using the create_publisher function of the parent:
  // "topic" the name of the topic
  // 10: the size of the queue for backup
  publisher_ = this->create_publisher<pubsub_custom_msg_in_pkg::msg::AddressBook>("topic", 10);
  
  // initializing the timer and binding it with the function timer_callback():
  timer_ = this->create_wall_timer(
    500ms, 
    std::bind(&PublisherNode::timer_callback, this)
  );
}

void PublisherNode::timer_callback(){
  // a new message:
  auto message = pubsub_custom_msg_in_pkg::msg::AddressBook();

  message.first_name = "John";
  message.last_name = "Doe";
  message.age = count_;
  message.address = "unknown";
  count_++;

  // printing the message on the logger:
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.first_name.c_str());
  // publishing the message on the topic:
  publisher_->publish(message);
}

