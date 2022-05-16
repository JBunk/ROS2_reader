#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

// to use 'ms' in our wall_timer:
using namespace std::chrono_literals;

// we pretend that this node is a 1 DOF robotarm
// It has two frames:
// 1) the base frame with as parent the world. The base can rotate.
// 2) the arm frame with as parent the base. The frame of the arm is at the end
//      of the arm and is static.

class MinimalTF2Broadcaster : public rclcpp::Node
{
public:
  MinimalTF2Broadcaster()
  : Node("MinimalTF2Broadcaster"), br(this), sbr(this), baseAngle_(0)
  {
    // we call the moveBase to create the base frame.
    // otherwise we cant make the arm frame a child of the base frame.
    moveBase();

    // we keep calling moveBase to update it over time (with fake movement)
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalTF2Broadcaster::moveBase, this));
    
    // create the arm frame:
    createArmFrame();
      
  }

private:

  void createArmFrame(){
    // create the message with our static transform:
    geometry_msgs::msg::TransformStamped static_transformStamped;

    // set the time:
    static_transformStamped.header.stamp = this->now();
    // the parent frame:
    static_transformStamped.header.frame_id = "base";
    // the child frame:
    static_transformStamped.child_frame_id = "arm";
    // the coordinates of the arm is in the current base frame:
    // we have a arm that is 20 units long in the direction of y.
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 20;
    static_transformStamped.transform.translation.z = 0.0;

    // Quarternion helps us with the angle data:
    tf2::Quaternion q;
    // the base angle in the World frame:
    q.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();

    // send transform
    sbr.sendTransform(static_transformStamped);
  }

  void moveBase()
  {
    // change the rotation of the base:
    baseAngle_++;

    // print a message in the terminal to look alive:
    RCLCPP_INFO(this->get_logger(), "sending a tf2 message: '%d'", baseAngle_);
  
    // create transform message
    // the transfrom stamped can also be created as a private member.
    // this will reduce duplicate code and allows for better performance.
    geometry_msgs::msg::TransformStamped transformStamped;

    // set the time:
    transformStamped.header.stamp = this->now();
    // the parent frame:
    transformStamped.header.frame_id = "world";
    // the child frame:
    transformStamped.child_frame_id = "base";
    // the coordinates of the base in the world frame:
    transformStamped.transform.translation.x = 50;
    transformStamped.transform.translation.y = 20;
    transformStamped.transform.translation.z = 0.0;

    // Quarternion helps us with the angle data:
    tf2::Quaternion q;
    // the base angle in the World frame:
    q.setRPY(0, 0, baseAngle_);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // send transform
    br.sendTransform(transformStamped);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  
  // declare the msg and the tf broadcasters for the base and arm frame
  // tf has it own broadcasters. There is no need to make the node a publisher!
  tf2_ros::TransformBroadcaster br;
  // if the frame doesnt change use the static broadcaster. It has better performance:
  // (but only if it doesnt change (often))
  tf2_ros::StaticTransformBroadcaster sbr;
  
  // some test data:
  size_t baseAngle_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalTF2Broadcaster>());
  rclcpp::shutdown();
  return 0;

  // test this node by running it and opening another terminal and
  // use the command: ros2 run tf2_ros tf2_echo world arm
  // you will see that the coordinates of the arm frame will change:
  // the arm frame is static to the base, but the base moves, so the arm
  // moves in respect to the world.

  // you can also test this by using the listener node that is also
  // provided within this package
}
