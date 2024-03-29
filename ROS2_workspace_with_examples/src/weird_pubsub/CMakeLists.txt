cmake_minimum_required(VERSION 3.5)
project(weird_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# make sure the folder inside /include had the same name as the project!
include_directories(include/${PROJECT_NAME}/)

add_executable(talkerAndListener src/mainWeirdNode.cpp src/WeirdNode.cpp)
ament_target_dependencies(talkerAndListener rclcpp std_msgs)

install(TARGETS
  talkerAndListener   
  DESTINATION lib/${PROJECT_NAME})

ament_package()
