#include "rclcpp/rclcpp.hpp"

class HelloNode : public rclcpp::Node
{
public:
    HelloNode();

private:
    // the function to be called by the timer:
    void timerCallback();

    // the timer:
    rclcpp::TimerBase::SharedPtr timer_;
};
