cmake_minimum_required(VERSION 3.8)
project(tether_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Specify the include directories
include_directories(
  include
)

add_executable(tether_control_node
  src/main.cpp
  src/tether_control.cpp
)
ament_target_dependencies(tether_control_node
  rclcpp 
  std_msgs)
  
add_executable(offboard_control_node
  src/offboard_control.cpp
)

ament_target_dependencies(offboard_control_node
  rclcpp 
  std_msgs
  px4_msgs)

install(
  DIRECTORY
    src
    launch
    models
    worlds
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  TARGETS
  tether_control_node
  offboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
