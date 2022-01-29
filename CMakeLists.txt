project(GoICP)

cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

add_library(goicp SHARED jly_goicp.cpp jly_3ddt.cpp matrix.cpp ConfigMap.cpp StringTokenizer.cpp)

add_executable(goicp-example jly_main.cpp)
target_link_libraries(goicp-example goicp)