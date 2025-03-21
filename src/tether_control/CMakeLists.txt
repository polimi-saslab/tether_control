cmake_minimum_required(VERSION 3.8)
project(tether_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ignition-cmake2 REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

# find_package(ignition-gazebo6 REQUIRED)
# add_library(tether_control SHARED CommandActor.cc)
# set_property(TARGET tether_control PROPERTY CXX_STANDARD 17)
# target_link_libraries(tether_control
#   PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#   PRIVATE ignition-gazebo6::ignition-gazebo6)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    src
    launch
    models
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/attach_tether.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
