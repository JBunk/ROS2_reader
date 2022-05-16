#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class WeirdNode : public rclcpp::Node
{
public:
    WeirdNode(const std::string & name);
  
private:
    void publishMessage(std::string content) const;

    // the function called everytime we receive a message from the topic:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

    // publisher with message type std_msgs::msg::String :
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    // the subscription:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
