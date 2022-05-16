#include "rclcpp/rclcpp.hpp"
// the msg-type from the custom_interfaces packages.
// AddressBook.msg becomes address_book.hpp
#include "custom_interfaces/msg/address_book.hpp" 

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode();

private:
    // the function called everytime we receive a message from the topic:
    void topic_callback(const custom_interfaces::msg::AddressBook::SharedPtr msg) const;

    // the subscription:
    // <message type> in this case our custom msg: custom_interfaces::msg::AddressBook
    rclcpp::Subscription<custom_interfaces::msg::AddressBook>::SharedPtr subscription_;
};
