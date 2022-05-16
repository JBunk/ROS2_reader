#include "rclcpp/rclcpp.hpp"
// msg/AddressBook.msg becomes msg/address_book.hpp:
#include "pub_without_spinning/msg/address_book.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode();

private:
    // the function called everytime we receive a message from the topic:
    void topic_callback(const pub_without_spinning::msg::AddressBook::SharedPtr msg) const;

    // the subscription:
    rclcpp::Subscription<pub_without_spinning::msg::AddressBook>::SharedPtr subscription_;
};
