cmake_minimum_required(VERSION 3.8)
project(tether_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include 
  SYSTEM ${Eigen3_INCLUDE_DIRS})

set(TETHER_CONTROL_SOURCES
  src/TetherControl/tether_control.cpp
  src/TetherControl/utils.cpp
  src/TetherControl/callbacks.cpp
  src/TetherControl/px4_publishers.cpp
  src/TetherControl/visualization.cpp
)

add_executable(tether_control_node
  src/main.cpp
  ${TETHER_CONTROL_SOURCES}
)

ament_target_dependencies(tether_control_node
  Eigen3
  geometry_msgs
  px4_msgs
  rclcpp 
  sensor_msgs
  std_msgs
  tf2
  tf2_ros)

####################### TETHER MODEL #######################

add_executable(tether_model_node
  src/tether_model.cpp
)

ament_target_dependencies(tether_model_node
  Eigen3
  geometry_msgs
  px4_msgs
  rclcpp 
  sensor_msgs
  std_msgs
  tf2)

  #########################################################

install(
  DIRECTORY
    config
    gz_plugin
    launch
    src
  DESTINATION
    share/${PROJECT_NAME}/
)

####################### GZ PLUGIN #######################
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(
  force_plugin 
  SHARED 
  gz_plugin/force_plugin.cpp
)

target_compile_features(force_plugin PUBLIC cxx_std_17)
target_include_directories(force_plugin PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/gz_plugin/include>
  $<INSTALL_INTERFACE:include/force_plugin>
)

ament_target_dependencies(
  force_plugin PUBLIC
  geometry_msgs
  rclcpp
)

target_link_libraries(force_plugin PUBLIC
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(
  TARGETS force_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
  tether_control_node
  tether_model_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
