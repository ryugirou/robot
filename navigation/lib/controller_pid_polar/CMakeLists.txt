cmake_minimum_required(VERSION 2.8.3)
add_library(controller_pid_polar STATIC controller_pid_polar.cpp)
target_include_directories(controller_pid_polar PUBLIC ${PROJECT_SOURCE_DIR}/lib/pid_position)