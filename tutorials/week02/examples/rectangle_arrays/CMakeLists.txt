# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.1)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (classes)

# This is added to c++11 features
set(CMAKE_CXX_STANDARD 11)

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
add_executable(rand_array_class rand_array_class.cpp randomarraygenerator.cpp)
add_executable(rand_rectangles rand_rectangles.cpp rectangle.cpp)
add_executable(rectangle_eg rectangle_main.cpp rectangle.cpp)

