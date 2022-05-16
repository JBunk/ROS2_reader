#include "rclcpp/rclcpp.hpp"
// srv/AddThreeInts.srv becomes srv/add_three_ints.hpp:
#include "srvcli_custom_srv_in_pkg/srv/add_three_ints.hpp"

class ClientNode : public rclcpp::Node
{
public:
    ClientNode();
  
private:
    // function that makes the request:
    void makeRequest();
    
    // function that handles the response:
    void handleResponse(rclcpp::Client<srvcli_custom_srv_in_pkg::srv::AddThreeInts>::SharedFuture future);

    rclcpp::Client<srvcli_custom_srv_in_pkg::srv::AddThreeInts>::SharedPtr client_;
};
