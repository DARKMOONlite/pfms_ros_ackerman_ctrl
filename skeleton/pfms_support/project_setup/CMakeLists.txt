cmake_minimum_required(VERSION 2.8.12)
project(project_setup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestGoal.srv
  DetectParking.srv
  FaceGoal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
##  LIBRARIES msgs_and_srvs
#  CATKIN_DEPENDS grid_map_msgs nmea_msgs
##  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)



## Declare a C++ executable
add_executable(TBK src/TBK.cpp)
add_executable(odo_republish src/odo_republish.cpp)
add_executable(laser_republish src/laser_republish.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(TBK ${catkin_LIBRARIES})
target_link_libraries(odo_republish ${catkin_LIBRARIES})
target_link_libraries(laser_republish ${catkin_LIBRARIES})
