#include "rclcpp/rclcpp.hpp"
#include "ServiceNode.h"

// placeholders for the arguments
using std::placeholders::_1;
using std::placeholders::_2;

ServiceNode::ServiceNode(): Node("ServiceNode"){
    // initialise the service
    // <srv-message-type> in this case custom_interfaces::srv::AddTwoInts
    // "add_three_ints" the name of the topic
    // and a pointer to the function to be called when a service message is received.
    service_ = this->create_service<srvcli_libellebil::srv::Word>(
            "words", 
            std::bind(&ServiceNode::handleService, this, _1, _2));
}

std::string ServiceNode::esrever(std::string word){
    for(size_t i=0; i<word.size()/2; i++){    
        std::swap(word[i], word[word.size()-i-1]);
    }
    return word;
}


void ServiceNode::handleService(const std::shared_ptr<srvcli_libellebil::srv::Word::Request> request,
          std::shared_ptr<srvcli_libellebil::srv::Word::Response> response)
{
  std::string outputWord = esrever(request->input_word);

  bool isIt = outputWord == request->input_word;

  response->output_word = outputWord;
  response->is_it = isIt;

}