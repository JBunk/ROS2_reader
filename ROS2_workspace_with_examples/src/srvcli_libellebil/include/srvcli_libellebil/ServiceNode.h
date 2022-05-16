#include "rclcpp/rclcpp.hpp"
#include "srvcli_libellebil/srv/word.hpp"

class ServiceNode : public rclcpp::Node
{
public:
  ServiceNode();
  
private:
    // function to be called by the service.
    std::string esrever(std::string word);

    void handleService(
        const std::shared_ptr<srvcli_libellebil::srv::Word::Request> request,
        std::shared_ptr<srvcli_libellebil::srv::Word::Response> response
    );

    rclcpp::Service<srvcli_libellebil::srv::Word>::SharedPtr service_;
};
