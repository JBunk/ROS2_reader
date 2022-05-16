#include "rclcpp/rclcpp.hpp"
#include "srvcli_libellebil/srv/word.hpp"

class ClientNode : public rclcpp::Node
{
public:
    ClientNode();
  
private:
    // function that makes the request:
    void makeRequest(const std::string & input);
    
    // function that handles the response:
    void handleResponse(rclcpp::Client<srvcli_libellebil::srv::Word>::SharedFuture future);

    rclcpp::Client<srvcli_libellebil::srv::Word>::SharedPtr client_;
};
