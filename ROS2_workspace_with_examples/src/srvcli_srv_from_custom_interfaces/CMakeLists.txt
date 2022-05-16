cmake_minimum_required(VERSION 3.5)
# make sure this is also the name in package.xml and the name of the folder
project(srvcli_srv_from_custom_interfaces)

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
find_package(custom_interfaces REQUIRED)

# make sure the folder inside /include had the same name as the project!
include_directories(include/${PROJECT_NAME}/)

# add executables (the nodes):
add_executable(server src/mainService.cpp src/ServiceNode.cpp)
ament_target_dependencies(server rclcpp custom_interfaces)

add_executable(client src/mainClient.cpp src/ClientNode.cpp)
ament_target_dependencies(client rclcpp custom_interfaces)

# let ros2 find the executables:
install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
