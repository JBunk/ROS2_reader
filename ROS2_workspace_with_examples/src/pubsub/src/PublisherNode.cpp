#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "PublisherNode.h"

using namespace std::chrono_literals; // allows us to use `ms'.

PublisherNode::PublisherNode(): Node("PublisherNode"), count_(0){
  // initializing the publisher using the create_publisher function of the parent:
  // "topic" the name of the topic
  // 10: the size of the queue
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  
  // initializing the timer and binding it with the function timer_callback():
  timer_ = this->create_wall_timer(
    500ms, 
    std::bind(&PublisherNode::timer_callback, this)
  );
}

void PublisherNode::timer_callback(){
  // a new message:
  auto message = std_msgs::msg::String();
  // adding data:
  count_++;
  message.data = "Hello, world! " + std::to_string(count_);
  // printing the message on the logger (not necessary for publishing):
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  // publishing the message on the topic:
  publisher_->publish(message);
}

