cmake_minimum_required(VERSION 3.15)
project(XQ4AI)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})

#2.Pro
file(GLOB XQ4SRC LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/*.h)
add_executable(XQ4AI CMakeLists.txt ${XQ4SRC} XQ4AI.cpp)