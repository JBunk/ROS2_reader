#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // variabele to count the number of message we have sent:
    size_t count_;
};
