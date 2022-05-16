cmake_minimum_required(VERSION 3.5)
project(hello)

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

# make sure the folder inside /include had the same name as the project!
include_directories(include/${PROJECT_NAME}/)

# add the executable
add_executable(greeter src/mainGreeter.cpp src/HelloNode.cpp)
ament_target_dependencies(greeter rclcpp)

# Making ROS 2 able to find our package:
install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
