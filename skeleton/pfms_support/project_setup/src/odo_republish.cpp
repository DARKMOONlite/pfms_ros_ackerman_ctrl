#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <string>

class OdoRepublish
{
private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
public:
  OdoRepublish()
  {

    ros::NodeHandle pn("~");
    std::string robot;
    pn.param<std::string>("robot", robot, "robot");
    ROS_INFO_STREAM("robot name:" << robot);
    std::string original_topic = robot + "/fix_odom";
    std::string new_topic = robot + "/odom";
    sub_ = n_.subscribe(original_topic, 1000, &OdoRepublish::odoCallback,this);
    pub_ = n_.advertise<nav_msgs::Odometry>(new_topic, 1000);
  }
  void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      nav_msgs::Odometry odo;
      odo.header.seq = msg->header.seq;
      odo.header.stamp.sec = msg->header.stamp.sec;
      odo.header.stamp.nsec = msg->header.stamp.nsec;
      odo.header.frame_id = msg->header.frame_id;
      odo.header.frame_id.erase(0,1); // Remove leading '/'
      odo.pose = msg->pose;

      // Publish the message
      pub_.publish(odo);

   }
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "odo_republish");

  OdoRepublish odoRepublish;

  ros::spin();

  return 0;
}
