cmake_minimum_required(VERSION 3.5)
project(rviz_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# make sure the folder inside /include had the same name as the project!
include_directories(include/${PROJECT_NAME}/)

add_executable(statePublisher src/state_publisher.cpp)
ament_target_dependencies(statePublisher rclcpp std_msgs geometry_msgs tf2 tf2_ros sensor_msgs)


install(TARGETS
  statePublisher
  DESTINATION lib/${PROJECT_NAME})

# Install launch and urdf (rviz) files.
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
