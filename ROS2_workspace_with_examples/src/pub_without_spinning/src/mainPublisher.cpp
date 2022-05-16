#include "rclcpp/rclcpp.hpp"
#include "pub_without_spinning/msg/address_book.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_node");
    auto pub = node->create_publisher<pub_without_spinning::msg::AddressBook>("topic", 10);

    auto message = pub_without_spinning::msg::AddressBook();
    message.first_name = "John";
    message.last_name = "Doe";
    message.age = 0;
    message.address = "unknown";

    pub->publish(message);
    for(int i=0; i<1000000000; i++){} // wait some time
    message.age++;
    pub->publish(message);
    for(int i=0; i<1000000000; i++){} // wait some time
    message.age++;
    pub->publish(message);
    for(int i=0; i<1000000000; i++){} // wait some time
    message.age++;
    pub->publish(message);
    for(int i=0; i<1000000000; i++){} // wait some time
    message.age++;
    pub->publish(message);

    return 0;
}
