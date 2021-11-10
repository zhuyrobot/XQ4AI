cmake_minimum_required(VERSION 3.15)
project(XQ4AI)

#1.Cfg
set(COMMON_CMAKE ${CMAKE_SOURCE_DIR}/common.cmake CACHE STRING "Common cmake file")
if(NOT EXISTS ${COMMON_CMAKE}) 
message(FATAL_ERROR "${COMMON_CMAKE} not exist") 
endif()
include(${COMMON_CMAKE})

#2.XQ4
file(GLOB sources LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/*)
file(GLOB XQ4SDK LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/XQ4SDK/*)

#3.RPLidar
include_directories(${CMAKE_SOURCE_DIR}/RPLSDK/include)
include_directories(${CMAKE_SOURCE_DIR}/RPLSDK/src)
file(GLOB RPLSDK LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/RPLSDK/*)
file(GLOB RPLSDK_INCS LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/RPLSDK/include/*)
file(GLOB RPLSDK_SRCS LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/RPLSDK/src/*)
file(GLOB RPLSDK_SRCS_hal LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/RPLSDK/src/hal/*)
file(GLOB RPLSDK_SRCS_arch_linux LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/RPLSDK/src/arch/linux/*)
file(GLOB RPLSDK_SRCS_arch_macos LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/RPLSDK/src/arch/macOS/*)
file(GLOB RPLSDK_SRCS_arch_win32 LIST_DIRECTORIES false ${CMAKE_SOURCE_DIR}/RPLSDK/src/arch/win32/*)
set(RPLSDK_SRCS_arch ${RPLSDK_SRCS_arch_macos})

if(MSVC)
source_group("sources" FILES ${sources})
source_group("XQ4SDK" FILES ${XQ4SDK})
source_group("RPLSDK" FILES ${RPLSDK})
source_group("RPLSDK/include" FILES ${RPLSDK_INCS})
source_group("RPLSDK/src" FILES ${RPLSDK_SRCS})
source_group("RPLSDK/src/hal" FILES ${RPLSDK_SRCS_hal})
source_group("RPLSDK/src/arch/win32" FILES ${RPLSDK_SRCS_arch_win32})
set(RPLSDK_SRCS_arch ${RPLSDK_SRCS_arch_win32})
endif(MSVC)

if(UNIX)
set(RPLSDK_SRCS_arch ${RPLSDK_SRCS_arch_linux})
endif(UNIX)

add_executable(${PROJECT_NAME} ${sources} ${XQ4SDK}
 ${RPLSDK} ${RPLSDK_INCS} ${RPLSDK_SRCS} 
 ${RPLSDK_SRCS_hal} ${RPLSDK_SRCS_arch})
