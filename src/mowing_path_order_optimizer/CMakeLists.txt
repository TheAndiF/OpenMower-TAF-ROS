cmake_minimum_required(VERSION 3.0.2)
project(mowing_path_order_optimizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  mbf_msgs
  nav_msgs
  roscpp
  slic3r_coverage_planner
  std_msgs
)

add_service_files(
  FILES
  OptimizePaths.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  mbf_msgs
  nav_msgs
  slic3r_coverage_planner
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib geometry_msgs mbf_msgs message_runtime nav_msgs roscpp slic3r_coverage_planner std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mowing_path_order_optimizer src/mowing_path_order_optimizer.cpp)
add_dependencies(mowing_path_order_optimizer ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mowing_path_order_optimizer ${catkin_LIBRARIES})

install(TARGETS mowing_path_order_optimizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
