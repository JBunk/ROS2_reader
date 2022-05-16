#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class StatePublisher : public rclcpp::Node
{
public:
  StatePublisher(): Node("statePublisher"), broadcaster(this), joint_pub(){
        
        joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
        
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "axis";
        
        // initializing the timer and binding it with the function timer_callback():
        timer_ = this->create_wall_timer(
            500ms, 
            std::bind(&StatePublisher::timer_callback, this)
        );


}
  
private:
    // function to be called by the timer.
    void timer_callback(){
        //update joint_state
        joint_state.header.stamp = this->now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] ="swivel";
        joint_state.position[0] = swivel;
        joint_state.name[1] ="tilt";
        joint_state.position[1] = tilt;
        joint_state.name[2] ="periscope";
        joint_state.position[2] = height;


        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = this->now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;

        // Quarternion helps us with the angle data:
        tf2::Quaternion q;
        // the base angle in the World frame:
        q.setRPY(0, 0, angle+M_PI/2);
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();

        //send the joint state and transform
        joint_pub->publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state
        tilt += tinc;
        if (tilt<-.5 || tilt>0) tinc *= -1;
        height += hinc;
        if (height>.2 || height<0) hinc *= -1;
        swivel += degree;
        angle += degree/4;

    }

    // the timer:
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::TransformBroadcaster broadcaster;

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::msg::TransformStamped odom_trans;
    sensor_msgs::msg::JointState joint_state;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;

    // iets met rosidl_message_type_support_t? huidige error
}