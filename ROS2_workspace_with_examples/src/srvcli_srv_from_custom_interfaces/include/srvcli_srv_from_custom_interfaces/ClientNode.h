#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_three_ints.hpp"

class ClientNode : public rclcpp::Node
{
public:
    ClientNode();
  
private:
    // function that makes the request:
    void makeRequest();
    
    // function that handles the response:
    void handleResponse(rclcpp::Client<custom_interfaces::srv::AddThreeInts>::SharedFuture future);

    rclcpp::Client<custom_interfaces::srv::AddThreeInts>::SharedPtr client_;
};
