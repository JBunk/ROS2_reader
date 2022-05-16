cmake_minimum_required(VERSION 3.5)
project(tf2_example)

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

add_executable(talker src/frameBroadcaster.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)

add_executable(listener src/frameListener.cpp)
ament_target_dependencies(listener rclcpp std_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
