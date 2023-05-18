# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Change project name
project (a1_skeleton)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Include the location of pfms headers
include_directories(/usr/local/include/pfms)
# Link to location of pfms libraries
link_directories(/usr/local/lib/pfms)

include_directories(/opt/pfms/include)
link_directories(/opt/pfms/lib)

# CMake will create your library, add the cpp files needed
#add_library(ranger )

# CMake will create the cell library that you will need 
#add_library(cell cell.cpp)

# CMake will create executable add the cpp files needed
#add_executable(assignment1 )


# Cmake will link against the libraries in final step
#target_link_libraries(assignment1 ranger cell)
