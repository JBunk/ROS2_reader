#include "rclcpp/rclcpp.hpp"
#include <chrono>

class HelloNode : public rclcpp::Node
{
public:
    HelloNode() : Node("HelloNode")
    {
        //initialisation of the timer:
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&HelloNode::timerCallback, this));
    }
    
private:
    // the function to be called by the timer:
    void timerCallback()
    {
        // print "Hello World!" in the logger:
        RCLCPP_INFO(this->get_logger(), "Hello World!");
    }
    // the timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
