# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (convar_starter)

find_package (Threads)


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

# CMake will now create the Makefiles that are needed to make an executable "test" that requires "main.cpp"
add_executable(example main.cpp)

#target_link_libraries (test ${CMAKE_THREAD_LIBS_INIT})
