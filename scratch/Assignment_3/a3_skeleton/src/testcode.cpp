/*! \mainpage Sebastian Schroder's PFMS Assignment: UGV
 *
 * \section Use the Tabs at the top to search for related information
 *  
 * 
 * \section Running Running The Code: <br>
 * 1st: run roscore <br>
 * 2nd: run Gazebo Simulator (My Simulation Crashes 9/10 times, IDK why) <br>
 * 3rd: run "rosrun a3_skeleton a3_skeleton_test_code" <br>
 * 4th: input your choice of Goals to follow, a small snippet of goals has been provided within the test folder <br>
 *    This snippet showcases the robot following a set of goals and then stopping as the goal lies outside of the cones <br>
 *    goals can be inputted using rosrun a3_support goals_publisher _goals:= "goal file location" /goals:= /orange/goals <br>
 * 5th: Start/Stop the vehicle at any time by Creating a service request on /orange/mission data = 1 wil; start the vehicle, data = 0 will stop it. <br>
 *  rosservice call /orange/mission "data = true"
 * 
 * 
 * \section tests
 * 
 * A Couple Tests have been added to the utest file and utilise their respective bags located within <br>
 * the bag subfolder.  <br>
 * 1st: run roscore <br>
 * 2nd: Run with rosrun a3_skeleton a3_skeleton_test <br>
 *

 */



#include "ros/ros.h"
#include "sample.h"
#include <thread>
#include <mission.h>
#include <ackerman.h>
#include <controller.h>
#include <controllerinterface.h>
#include "sample.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation
#include "laserprocessing.h"

int main(int argc, char **argv)
{

std::cout << "Starting in main" << std::endl;
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "a3_skeleton");
 ROS_INFO_STREAM("Initialised");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on the function desired
   */
std::vector<ControllerInterface*> controllers;



 controllers.push_back(new Ackerman(nh));
Mission mission(controllers,nh); 
    ros::Subscriber test;


    mission.setMissionObjective(mission::BASIC);
   
//     std::vector<std::pair<pfms::geometry_msgs::Point,pfms::geometry_msgs::Point>> Cones;

//     pfms::geometry_msgs::Point left = {32,18};
//     pfms::geometry_msgs::Point right = {26,8};
//     std::pair<pfms::geometry_msgs::Point,pfms::geometry_msgs::Point> Point = {std::make_pair(left,right)};
    
// Cones.push_back(Point);

// ROS_INFO_STREAM("Cones Created");
//    std::thread t(&Mission::setGoals,mission,std::ref(Cones));
// ROS_INFO_STREAM("Published Cones");
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

 std::thread t(&Mission::run,&mission);
 



  ros::Rate loop_rate(100);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();

  }
  // ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  //  t.join();

  return 0;
}