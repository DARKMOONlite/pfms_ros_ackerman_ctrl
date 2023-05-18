#include <gtest/gtest.h>
#include <climits>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <mission.h>
#include <ackerman.h>
#include <controller.h>
#include <controllerinterface.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "laserprocessing.h"

int my_argc ;
char** myargv;

TEST(LaserProcessing,TestClosestPosition){

  //! The below code tests the laserprocessing class
  //! The data has been saved in a bag, that is opened and used.
  //! Unforttunately as we need to use a ConstPtr below, we can't make this
  //! a helper function

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("a3_skeleton");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/bag/";
  std::string file = path + "laser.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;

  //! The bag has all the messages, so we go through all of them to find the mesages we need
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    //! We will go through the bag and extract the laser scan and odometry
    //! We have to try to instatitate each message type

    if(m.getTopic() == "/orange/laser/scan"){
      if( laserScan == nullptr){
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
      }
    }
    if(m.getTopic() == "/ugv_odom"){
      if( odom == nullptr){
        odom = m.instantiate<nav_msgs::Odometry>();
      }
    }
    if ((laserScan != nullptr) && (odom != nullptr)){
      //! Now we have a laserScan and odometry so we can proceed
      //! We could also check here if we have High Intensity readings before abandoning the loop
      break;
    }
  }
  bag.close();



  ASSERT_NE(laserScan, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(odom, nullptr);

  ////////////////////////////////////////////
  // Our code is tested below


  //! Create an object of LaserProcessing class as we will use the public function of that object to run tests against
  LaserProcessing laserProcessing(*laserScan);
  geometry_msgs::Point point = laserProcessing.detectPositionClosest();

  // We check the closest point knowing that this is indeed the actual value, refer to quiz 5 for more examples
  EXPECT_NEAR(point.x,2,0.1);
  EXPECT_NEAR(point.y,-3.6,0.1);

}
TEST(LaserProcessing,Detect_All_Cones){
  std::string path = ros::package::getPath("a3_skeleton");
  path += "/test/bag/";
  std::string file = path + "laser.bag";

  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/orange/laser/scan"){
      if( laserScan == nullptr){
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
      }
    }
    if(m.getTopic() == "/ugv_odom"){
      if( odom == nullptr){
        odom = m.instantiate<nav_msgs::Odometry>();
      }
    }
    if ((laserScan != nullptr) && (odom != nullptr)){
      break;
    }
  }
  bag.close();


  LaserProcessing laserProcessing(*laserScan);
  auto Points = laserProcessing.detectPoints(180);
 
  EXPECT_EQ(Points.size(),7);
   Points = laserProcessing.detectPoints(120);

  EXPECT_EQ(Points.size(),5);
    Points = laserProcessing.detectPoints(90);
 
  EXPECT_EQ(Points.size(),4);

}

TEST(Goals,GoalReachable){
 std::string path = ros::package::getPath("a3_skeleton");
  path += "/test/bag/";
  std::string file = path + "goals.bag";

  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
  geometry_msgs::PoseArray::ConstPtr goals = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/orange/laser/scan"){
      if( laserScan == nullptr){
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
      }
    }
    if(m.getTopic() == "/orange/goals"){
      if( goals == nullptr){
        goals = m.instantiate<geometry_msgs::PoseArray>();
      }
    }
    if(m.getTopic() == "/ugv_odom"){
      if( odom == nullptr){
        odom = m.instantiate<nav_msgs::Odometry>();
      }
    }
    if ((goals != nullptr) && (odom != nullptr)&& (laserScan != nullptr)){
      break;
    }
  }
  bag.close(); 

  ASSERT_NE(laserScan, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(odom, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(goals,nullptr);//

  LaserProcessing laserProcessing(*laserScan);
  std::vector<geometry_msgs::Point> points = laserProcessing.detectPoints(90);

ros::init(my_argc,myargv,"GoogleTest");
  ros::NodeHandle nh;

 Ackerman Vehicle(nh);
auto test = goals->poses.at(0);
auto Goal = Vehicle.ConvertGoal2pfms(test);
auto test2 = goals->poses.at(1);
auto Goal2 = Vehicle.ConvertGoal2pfms(test2);
auto odo = Vehicle.ConvertOdo2pfms(*odom);

EXPECT_TRUE(Vehicle.WithinTriangle(Goal,points,odo));

EXPECT_FALSE(Vehicle.WithinTriangle(Goal2,points,odo));



}
TEST(Sonar,Avoid_Object_Cone){

std::string path = ros::package::getPath("a3_skeleton");
  path += "/test/bag/";
  std::string file = path + "sonar.bag";

  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
  sensor_msgs::Range::ConstPtr sonar = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/orange/laser/scan"){
      if( laserScan == nullptr){
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
      }
    }
    if(m.getTopic() == "/orange/sonar/range"){
      if( sonar == nullptr){
        sonar = m.instantiate<sensor_msgs::Range>();
      }
    }
    if(m.getTopic() == "/ugv_odom"){
      if( odom == nullptr){
        odom = m.instantiate<nav_msgs::Odometry>();
      }
    }
    if ((sonar != nullptr) && (odom != nullptr)&& (laserScan != nullptr)){
      break;
    }
  }
  bag.close(); 

  ASSERT_NE(laserScan, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(odom, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(sonar,nullptr);//
ros::init(my_argc,myargv,"GoogleTest2");
  ros::NodeHandle nh2;

 Ackerman Vehicle(nh2);
 Vehicle.getLaserScan(laserScan);
 Vehicle.OverrideRunningCheck(true); // This forces the vehicle to enter the running mode, which exhibits different properties
 EXPECT_EQ(Vehicle.ReadRunningCheck(),true); 

  Vehicle.getSonar(sonar);
  EXPECT_EQ(Vehicle.ReadRunningCheck(),true);

}

TEST(Sonar,Avoid_Object_Not_Cone){

std::string path = ros::package::getPath("a3_skeleton");
  path += "/test/bag/";
  std::string file = path + "sonar.bag";

  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
  sensor_msgs::Range::ConstPtr sonar = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/orange/laser/scan"){
      if( laserScan == nullptr){
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
      }
    }
    if(m.getTopic() == "/orange/sonar/range"){
      if( sonar == nullptr){
        sonar = m.instantiate<sensor_msgs::Range>();
      }
    }
    if(m.getTopic() == "/ugv_odom"){
      if( odom == nullptr){
        odom = m.instantiate<nav_msgs::Odometry>();
      }
    }
    if ((sonar != nullptr) && (odom != nullptr)&& (laserScan != nullptr)){
      break;
    }
  }
  bag.close(); 

  ASSERT_NE(laserScan, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(odom, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(sonar,nullptr);//
ros::init(my_argc,myargv,"GoogleTest3");
  ros::NodeHandle nh2;

 Ackerman Vehicle(nh2);

 Vehicle.OverrideRunningCheck(true); // This forces the vehicle to enter the running mode, which exhibits different properties
 EXPECT_EQ(Vehicle.ReadRunningCheck(),true); 

  Vehicle.getSonar(sonar);
  EXPECT_EQ(Vehicle.ReadRunningCheck(),false); //This should be false as the vehicle should've stopped

}




int main(int argc, char **argv) {
  my_argc = argc;
  myargv = argv;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
