#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

// to use 'ms' in our wall_timer:
using namespace std::chrono_literals;

// This nodes listens to where the arm frame is relative to the world.
// this example is based on the tf2 listener tutorial and:
// https://github.com/ros2/geometry2/blob/ros2/tf2_ros/src/tf2_echo.cpp
class MinimalTF2Listener : public rclcpp::Node
{
public:
  MinimalTF2Listener()
  : Node("MinimalTF2Listener"), tfBuffer(this->get_clock()), tfListener(tfBuffer)
  {

    // we keep calling moveBase to update it over time (with fake movement)
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalTF2Listener::ListenAndPrint, this));
       
  }

private:

  void ListenAndPrint()
  {
    // create transform message
    geometry_msgs::msg::TransformStamped transformStamped;

    // try to get the coordinates of arm in the world frame:
    try{
      transformStamped = tfBuffer.lookupTransform("world", "arm", tf2::TimePoint());
    }
    // otherwise explain what went wrong and exit this function:
    catch (const tf2::TransformException & ex) {
      std::cout << "Failure at " << this->now().seconds() << std::endl;
      std::cout << "Exception thrown:" << ex.what() << std::endl;
      std::cout << "The current list of frames is:" << std::endl;
      std::cout << tfBuffer.allFramesAsString() << std::endl;
      return;
    }

    // print the coordinates of arm:
    RCLCPP_INFO(this->get_logger(), "The arm is at: (%.5f,%.5f,%.5f)", 
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.z);
  }
  
  // the timer:
  rclcpp::TimerBase::SharedPtr timer_;
  
  // declare the buffer and the listener 
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalTF2Listener>());
  rclcpp::shutdown();
  return 0;

  // test this node by running it and running the broadcasting node that is
  // in this package
}
