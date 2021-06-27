cmake_minimum_required(VERSION 3.15)
project(XQ4Server)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})

#2.Pro
add_executable(XQ4RC CMakeLists.txt XQ4IO.h XQ4RC.cpp)
add_executable(XQ4ROS2Server XQ4IO.h XQ4ROS2Server.cpp)
add_executable(XQ4ROS2Client XQ4IO.h XQ4ROS2Client.cpp)
add_executable(XQ4SPQT XQ4SPQT.cpp)