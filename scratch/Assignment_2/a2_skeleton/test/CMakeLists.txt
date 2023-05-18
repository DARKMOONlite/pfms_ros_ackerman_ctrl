# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
set(PROJECT_NAME a2_tests)
project (${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Include the location of pfms headers
include_directories(/opt/pfms/include)
# Link to location of pfms libraries
link_directories(/opt/pfms/lib)

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

# This uses the complete student's library
aux_source_directory(.. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp")
message ( STATUS "Compiling student_lib with following files  ${SRC_LIST}" )
add_library(student_lib ${SRC_LIST})
target_link_libraries(student_lib pipes)

# Now we make the gtests
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

message("CHECK_PURE_VIRTUAL=${CHECK_PURE_VIRTUAL}")
if( CHECK_PURE_VIRTUAL )
    message("CHECKING PURE VIRTUAL")
    add_executable(pureVirtualTests test_pure_virtual.cpp)
    target_link_libraries(pureVirtualTests ${GTEST_LIBRARIES} pthread)
    target_link_libraries(pureVirtualTests student_lib)
endif()

## Three tests for the AUDI
add_executable(audiCheckOriginToDestination audiCheckOriginToDestination.cpp)
target_link_libraries(audiCheckOriginToDestination ${GTEST_LIBRARIES} linkcommand student_lib pthread pipes)

add_executable(audiReachGoal audiReachGoal.cpp)
target_link_libraries(audiReachGoal ${GTEST_LIBRARIES} linkcommand student_lib pthread)

add_executable(audiReachGoals audiReachGoals.cpp)
target_link_libraries(audiReachGoals ${GTEST_LIBRARIES} linkcommand student_lib pthread)

# Three tests for the QUADCOPTER
add_executable(quadcopterCheckOriginToDestination quadcopterCheckOriginToDestination.cpp)
target_link_libraries(quadcopterCheckOriginToDestination ${GTEST_LIBRARIES} student_lib linkcommand  pthread)

add_executable(quadcopterReachGoal quadcopterReachGoal.cpp)
target_link_libraries(quadcopterReachGoal ${GTEST_LIBRARIES} student_lib linkcommand  pthread)

add_executable(quadcopterReachGoals quadcopterReachGoals.cpp)
target_link_libraries(quadcopterReachGoals ${GTEST_LIBRARIES} student_lib linkcommand pthread)

add_executable(quadcopterGoalOrder quadcopterGoalOrderCheck.cpp)
target_link_libraries(quadcopterGoalOrder  ${GTEST_LIBRARIES} student_lib linkcommand pthread)

add_executable(audiGoalOrder audiGoalOrderCheck.cpp)
target_link_libraries(audiGoalOrder ${GTEST_LIBRARIES} student_lib linkcommand pthread)