cmake_minimum_required(VERSION 3.5)
# make sure this is also the name in package.xml and the name of the folder
project(srvcli_custom_srv_in_pkg)

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

#make the custom message type
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddThreeInts.srv"
 )

ament_export_dependencies(rosidl_default_runtime)

# make sure the folder inside /include had the same name as the project!
include_directories(include/${PROJECT_NAME}/)

# add executables (the nodes):
add_executable(server src/mainService.cpp src/ServiceNode.cpp)
ament_target_dependencies(server rclcpp)

add_executable(client src/mainClient.cpp src/ClientNode.cpp)
ament_target_dependencies(client rclcpp)

# let ros2 find the executables:
install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

# needed when using msg-types within the package:
rosidl_target_interfaces(server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
