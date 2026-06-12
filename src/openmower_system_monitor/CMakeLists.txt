cmake_minimum_required(VERSION 3.0.2)
project(openmower_system_monitor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xbot_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(system_monitor_node
  src/system_monitor_node.cpp
)

add_dependencies(system_monitor_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(system_monitor_node
  ${catkin_LIBRARIES}
)
