#include "rclcpp/rclcpp.hpp"
// srv/AddThreeInts.srv becomes srv/add_three_ints.hpp:
#include "srvcli_custom_srv_in_pkg/srv/add_three_ints.hpp"

class ServiceNode : public rclcpp::Node
{
public:
  ServiceNode();
  
private:
    // function to be called by the service.
    void add(
        const std::shared_ptr<srvcli_custom_srv_in_pkg::srv::AddThreeInts::Request> request,
        std::shared_ptr<srvcli_custom_srv_in_pkg::srv::AddThreeInts::Response> response
    );

    rclcpp::Service<srvcli_custom_srv_in_pkg::srv::AddThreeInts>::SharedPtr service_;
};
