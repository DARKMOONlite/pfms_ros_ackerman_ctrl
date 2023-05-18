# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Change project name
project (a1_skeleton)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

 include_directories(${PROJECT_NAME})
 include_directories(${PROJECT_SOURCE_DIR})
link_directories(${PROJECT_NAME} ${PROJECT_SOURCE_DIR})
## Include the location of pfms headers
include_directories(/usr/local/include/pfms)
# Link to location of pfms libraries
link_directories(/usr/local/lib/pfms)

# set(HEADERS 
# ackerman.h 
# controller.h 
# controllerinterface.h
# mission.h
# missioninterface.h
# quadcopter.h
# )

## Include the location of pfms headers
include_directories(/opt/pfms/include)
link_directories(/opt/pfms/lib)

include_directories(../include)
include_directories(..)

find_library(pipes opt/pfms/lib)



#add_executable(main main.cpp ${HEADERS})
#add_executable(main main.cpp)
# CMake will create your library, add the cpp files needed
add_library(controller controller.cpp controller.h)
add_library(quadcopter quadcopter.cpp quadcopter.h)
add_library(ackerman ackerman.cpp ackerman.h)
add_library(mission mission.cpp mission.h)
# CMake will create the cell library that you will need 
#add_library(cell cell.cpp)

# CMake will create executable add the cpp files needed
#add_executable(assignment1 )
add_executable(main main.cpp )
#add_executable(MyTest MyTests.cpp ackerman.cpp controller quadcopter.cpp mission.cpp)
# Cmake will link against the libraries in final step
#target_link_libraries(assignment1 ranger cell)
target_link_libraries(main pipes controller quadcopter ackerman mission)
#target_link_libraries(MyTest pipes controller)
