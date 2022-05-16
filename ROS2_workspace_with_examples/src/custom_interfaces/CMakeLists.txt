cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

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
 
include_directories(include/${PROJECT_NAME}/)

#dependencie needed to create generate custom messages:
find_package(rosidl_default_generators REQUIRED)

# add here the custom messages, services and actions:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AddressBook.msg"
  "srv/AddThreeInts.srv"
  "action/Fibonacci.action"
 )

ament_package()
