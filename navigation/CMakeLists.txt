cmake_minimum_required(VERSION 2.8.3)
project(navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2
  tr
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(lib/controller_pid)
add_subdirectory(lib/pid_position)
add_subdirectory(lib/controller_pid_polar)

catkin_package(
  INCLUDE_DIRS lib/pid_position
  LIBRARIES pid_position
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf tf2
)

add_executable(trajectory_tracking src/trajectory_tracking.cpp)
target_include_directories(trajectory_tracking PRIVATE lib/controller_pid_polar)
target_link_libraries(trajectory_tracking ${catkin_LIBRARIES} controller_pid_polar pid_position)
add_dependencies(trajectory_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})