cmake_minimum_required(VERSION 3.8)
project(tether_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(rmw REQUIRED)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

include_directories(
  include 
  SYSTEM ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS})

add_library(apply_force_plugin SHARED src/apply_force_plugin.cpp)
set_property(TARGET apply_force_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(apply_force_plugin
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim8::gz-sim8)

add_executable(tether_control_node
  src/main.cpp
  src/tether_control.cpp
)

ament_target_dependencies(tether_control_node
  rclcpp 
  std_msgs
  Eigen3
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
  apply_force_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
