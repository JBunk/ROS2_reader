#include "rclcpp/rclcpp.hpp"
// msg/AddressBook.msg becomes msg/address_book.hpp:
#include "pubsub_custom_msg_in_pkg/msg/address_book.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode();

private:
    // the function called everytime we receive a message from the topic:
    void topic_callback(const pubsub_custom_msg_in_pkg::msg::AddressBook::SharedPtr msg) const;

    // the subscription:
    rclcpp::Subscription<pubsub_custom_msg_in_pkg::msg::AddressBook>::SharedPtr subscription_;
};
