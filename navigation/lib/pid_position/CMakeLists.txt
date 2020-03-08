cmake_minimum_required(VERSION 2.8.3)
add_library(pid_position pid_position.cpp)

# INSTALL(TARGETS pid_position LIBRARY DESTINATION lib)
# target_include_directories(pid_position INTERFACE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/pid_position>
#     $<INSTALL_INTERFACE:include>) 
# INSTALL(DIRECTORY lib/pid_position/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )