# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz3_a2)

set(BUILD_TESTS ON) 
# Needed to use threads
find_package (Threads)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -no-pie -pthread")

# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(RADAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${RADAR_LIB_HOME})
message ( STATUS "Looking for libray in ${RADAR_LIB_HOME}" )
link_directories(${RADAR_LIB_HOME})

add_library(analysis analysis.cpp)
target_link_libraries(analysis radar)

# Add executable
add_executable(${PROJECT_NAME}_ex main.cpp)

# Link the library
target_link_libraries(${PROJECT_NAME}_ex analysis)

##################################################################################################
# The below is our an example of our automated tets for marking
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON
# then run make as usual
# finally, tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
