# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.0)

# Change project name
project (a2_skeleton)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Include the location of pfms headers
include_directories(/opt/pfms/include)
# Link to location of pfms libraries
link_directories(/opt/pfms/lib)

# We supply a logger library to read/write Point3d to a file
add_library(logger logger.cpp)

## You NEED to add your libraries and also link them if needed (refer Week 04 for syntax)
#add_library(controllers )
#add_library(mission )
#target_link_libraries(controllers pipe)

# Sreate executable with  cpp files needed
add_executable(assignment2 main.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(assignment2 logger)
