cmake_minimum_required(VERSION 2.8.13)
project(gazebo_tf)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg gazebo_msgs geometry_msgs tf)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Include the headers and library directory needed
include_directories(/opt/pfms/include)
link_directories(/opt/pfms/lib)

#Create executables
add_executable(goals src/goals.cpp)
target_link_libraries(goals ${catkin_LIBRARIES} pipes)

add_executable(uav_comms src/uav_comms.cpp)
target_link_libraries(uav_comms ${catkin_LIBRARIES} pipes)

add_executable(uav_odo src/uav_odo.cpp)
target_link_libraries(uav_odo ${catkin_LIBRARIES} pipes)

add_executable(ugv_comms src/ugv_comms.cpp)
target_link_libraries(ugv_comms ${catkin_LIBRARIES} pipes)

add_executable(ugv_odo src/ugv_odo.cpp)
target_link_libraries(ugv_odo ${catkin_LIBRARIES} pipes)

add_executable(gazebo_connect src/gazebo_connect.cpp)
target_link_libraries(gazebo_connect ${catkin_LIBRARIES})

add_executable(test_odo src/test_odo.cpp)
target_link_libraries(test_odo ${catkin_LIBRARIES} pipes  rt pthread )

