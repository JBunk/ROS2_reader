Run this package with in a terminal:
ros2 launch rviz_example demo.launch.py

and start the rviz in another terminal with:
rviz2 -d ~/location/of/your/workspace/src/rviz_example/share/rviz_example/r2d2.rviz

this will start rviz with the rviz configuration file
(Sometimes it doesnt load the rviz configuration file. Then you wont see a robot.
Fix this by manually loading the rviz configuration file.)


the package 'robot_state_publisher' publishes the urdf robot file to tf2.
See: http://wiki.ros.org/robot_state_publisher  (ros 1 documentation, but the package hasnt really changed)


in the state_publisher.cpp we move the frame 'axis' relative to 'odom'. 'odom' is
our hook with the world. 'axis' is the parent of most joints of the robot (see 
/urdf/r2d2.urdf.xml) 

JOINT STATES and TF2 are two different things. Quick summary, but better is to
read the documentation.

JOINT STATES:
This is a message that holds data to describe the state of a set of torque controlled joints.

The state of each joint (revolute or prismatic) is defined by:
  * the position of the joint (rad or m),
  * the velocity of the joint (rad/s or m/s) and 
  * the effort that is applied in the joint (Nm or N).

TF2:
gives the coordinates of something (could be a joint) in the coordinate world of
something else.
