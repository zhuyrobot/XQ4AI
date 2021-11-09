cmake_minimum_required(VERSION 3.15)
project(XQ4AI)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})

#2.Pro
file(GLOB sources LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/*)
file(GLOB XQ4SDK LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/XQ4SDK/*)
source_group("sources" FILES ${sources})
source_group("XQ4SDK" FILES ${XQ4SDK})
add_executable(${PROJECT_NAME} ${XQ4SDK} ${sources})