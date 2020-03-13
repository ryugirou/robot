cmake_minimum_required(VERSION 2.8.3)
project(navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(lib/controller_pid)
add_subdirectory(lib/pid_position)

catkin_package(
  INCLUDE_DIRS lib/pid_position
  LIBRARIES pid_position
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf tf2
)

add_executable(trajectory_tracking src/trajectory_tracking.cpp)
target_include_directories(trajectory_tracking PRIVATE lib/controller_pid)
target_link_libraries(trajectory_tracking ${catkin_LIBRARIES} controller_pid pid_position)
add_dependencies(trajectory_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_tracking2 src/trajectory_tracking2.cpp)
target_include_directories(trajectory_tracking2 PRIVATE lib/controller_pid)
target_link_libraries(trajectory_tracking2 ${catkin_LIBRARIES} controller_pid pid_position)
add_dependencies(trajectory_tracking2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})