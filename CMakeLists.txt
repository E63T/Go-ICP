project(GoICP)

cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
find_package(PCL 1.11 REQUIRED COMPONENTS common registration)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

add_library(goicp SHARED jly_goicp.cpp jly_3ddt.cpp)
target_link_libraries(goicp ${PCL_LIBRARIES})

add_executable(goicp-example jly_main.cpp)
target_link_libraries(goicp-example goicp ${PCL_LIBRARIES})