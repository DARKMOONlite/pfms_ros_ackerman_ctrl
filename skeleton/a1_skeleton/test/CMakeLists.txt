# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
set(PROJECT_NAME a1_tests)
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

add_executable(audiTests test_audi.cpp)
target_link_libraries(audiTests ${GTEST_LIBRARIES} pipes student_lib pthread)

add_executable(quadcopterTests test_quadcopter.cpp)
target_link_libraries(quadcopterTests ${GTEST_LIBRARIES} pipes student_lib pthread)

add_executable(missionTests test_mission.cpp)
target_link_libraries(missionTests ${GTEST_LIBRARIES} pipes student_lib pthread)

