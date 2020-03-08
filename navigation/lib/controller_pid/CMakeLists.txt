cmake_minimum_required(VERSION 2.8.3)
add_library(controller_pid STATIC controller_pid.cpp)
target_include_directories(controller_pid PUBLIC ${PROJECT_SOURCE_DIR}/lib/pid_position)