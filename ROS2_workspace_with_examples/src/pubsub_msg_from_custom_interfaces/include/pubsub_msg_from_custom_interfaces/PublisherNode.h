#include "rclcpp/rclcpp.hpp"
// the msg-type from the custom_interfaces packages.
// AddressBook.msg becomes address_book.hpp
#include "custom_interfaces/msg/address_book.hpp" 

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode();
  
private:
    // function to be called by the timer.
    void timer_callback();

    // the timer:
    rclcpp::TimerBase::SharedPtr timer_;

    // publisher with message type std_msgs::msg::String :
    rclcpp::Publisher<custom_interfaces::msg::AddressBook>::SharedPtr publisher_;
    size_t count_;
};
