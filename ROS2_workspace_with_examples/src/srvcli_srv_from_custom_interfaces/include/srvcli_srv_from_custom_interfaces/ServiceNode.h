#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_three_ints.hpp"

class ServiceNode : public rclcpp::Node
{
public:
  ServiceNode();
  
private:
    // function to be called by the service.
    void add(
        const std::shared_ptr<custom_interfaces::srv::AddThreeInts::Request> request,
        std::shared_ptr<custom_interfaces::srv::AddThreeInts::Response> response
    );

    rclcpp::Service<custom_interfaces::srv::AddThreeInts>::SharedPtr service_;
};
