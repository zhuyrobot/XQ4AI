cmake_minimum_required(VERSION 3.15)
project(XQ4Server)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})

#2.Pro
add_executable(SimXQ4 CMakeLists.txt XQ4IO.h SimXQ4.cpp)
add_executable(XQ4IOUI XQ4IOUI.cpp)
add_executable(XQ4Robot XQ4Robot.cpp)
add_executable(XQ4RobotRC XQ4RobotRC.cpp)
add_executable(XQ4ZDeprecatedQTPort XQ4ZDeprecatedQTPort.cpp)