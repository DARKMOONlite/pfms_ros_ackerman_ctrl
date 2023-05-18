cmake_minimum_required (VERSION 2.6)

project (critical_sections)

# Needed to use threads
find_package (Threads)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

add_executable(ex02 main.cpp)
