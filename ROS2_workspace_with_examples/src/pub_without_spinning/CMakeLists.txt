cmake_minimum_required(VERSION 3.5)
project(pub_without_spinning)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

#make the custom message type
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AddressBook.msg"
 )

ament_export_dependencies(rosidl_default_runtime)

# make sure the folder inside /include had the same name as the project!
include_directories(include/${PROJECT_NAME}/)

add_executable(talker src/mainPublisher.cpp)
ament_target_dependencies(talker rclcpp)

add_executable(listener src/mainSubscriber.cpp src/SubscriberNode.cpp)
ament_target_dependencies(listener rclcpp)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# needed when using msg-types within the package:
rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
