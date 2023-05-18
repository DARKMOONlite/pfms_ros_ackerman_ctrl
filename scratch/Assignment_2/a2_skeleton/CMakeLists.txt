# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.0)

# Change project name
set (PROJECT_NAME assignment2)
project(${PROJECT_NAME})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Include the location of pfms headers
include_directories(/opt/pfms/include)
# Link to location of pfms libraries
link_directories(/opt/pfms/lib)

# We supply a logger library to read/write Point3d to a file
add_library(logger logger.cpp)

## You NEED to add your libraries and also link them if needed (refer Week 04 for syntax)
# add_library(controllers)
# add_library(mission )

include_directories(../include)
include_directories(..)

# aux_source_directory(/ SRC_LIST)
# list(REMOVE_ITEM SRC_LIST "../main.cpp")
# message ( STATUS "Compiling student_lib with following files  ${SRC_LIST}" )
# add_library(student_lib ${SRC_LIST})
# target_link_libraries(student_lib pipes)


aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp")
add_library(student_lib ${SRC_LIST})
target_link_libraries(student_lib pipes)
# add_library(controller controller.cpp controller.h controllerinterface.h)
# add_library(quadcopter quadcopter.cpp quadcopter.h)
# add_library(ackerman ackerman.cpp ackerman.h)
# add_library(mission mission.cpp mission.h missioninterface.h)

# Sreate executable with  cpp files needed
add_executable(assignment2 main.cpp)
target_link_libraries(assignment2   pthread student_lib)
# target_link_libraries(assignment2 controller quadcopter ackerman mission)
# Cmake will link against the libraries in final step
target_link_libraries(assignment2 logger )
