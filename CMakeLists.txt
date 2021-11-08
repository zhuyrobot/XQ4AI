cmake_minimum_required(VERSION 3.15)
project(XQ4AI)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})

#2.Pro
add_executable(XQ4ROS CMakeLists.txt XQ4IO.h XQ4ROS.cpp)
add_executable(XQ4Studio CMakeLists.txt XQ4IO.h XQ4Studio.cpp)
add_executable(XQ4ZDeprecatedQTRC XQ4ZDeprecatedQTRC.cpp)