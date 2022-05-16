#include "rclcpp/rclcpp.hpp"
// msg/AddressBook.msg becomes msg/address_book.hpp:
#include "pubsub_custom_msg_in_pkg/msg/address_book.hpp"

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode();
  
private:
    // function to be called by the timer.
    void timer_callback();

    // the timer:
    rclcpp::TimerBase::SharedPtr timer_;

    // publisher with message type pubsub_custom_msg_in_pkg::msg::My_msg :
    rclcpp::Publisher<pubsub_custom_msg_in_pkg::msg::AddressBook>::SharedPtr publisher_;
    size_t count_;
};
