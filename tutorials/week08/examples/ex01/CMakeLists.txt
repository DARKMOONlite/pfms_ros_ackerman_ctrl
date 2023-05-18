# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Add project name
project (week08)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Include the location of pfms headers
include_directories(/opt/pfms/include)
# Link to location of pfms libraries
link_directories(/opt/pfms/lib)

# Find the library we are testing (and has been provided in the dep subdirectory)
# The library is called libstudent.so (student library)
set(SAMPLE_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${SAMPLE_LIB_HOME}/include)
link_directories(${SAMPLE_LIB_HOME}/lib)

add_executable(ex01 main.cpp)
# Link to the library, this presumes you have built three libraries (each with name below)
# If you only had one library then you can add a single library name
target_link_libraries(ex01 pipes student)

##################################################################################################
# The below is our an example tests for marking
# We have only provided 3 examples (one is the existing unit test we supplied)
# To enable testsing we instaed set BUILD_TEST to ON by default
set(BUILD_TESTS ON )

if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
