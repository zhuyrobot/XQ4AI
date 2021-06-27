###I.Global
cmake_minimum_required(VERSION 3.15)
project(ams_xq)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

###II.Denpens
if(UNIX)
file(GLOB ROS2_LIBS LIST_DIRECTORIES false /opt/ros/foxy/lib/*.so)
endif(UNIX)
if(WIN32)
file(GLOB ROS2_LIBS LIST_DIRECTORIES false D:/app/ros2/lib/*.lib)
endif(WIN32)
link_libraries(${ROS2_LIBS})
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} 
"XQ4Frame.msg"
"XQ4Serve.srv"
)

  
###III.Export
#ament_export_dependencies(rosidl_default_runtime)###???
ament_package()###make this project found by other projects using find_package
