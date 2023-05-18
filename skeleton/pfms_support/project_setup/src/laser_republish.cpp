#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <string>

class LaserRepublish
{
private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
public:
  LaserRepublish()
  {

    ros::NodeHandle pn("~");
    std::string robot;
    pn.param<std::string>("robot", robot, "robot");
    ROS_INFO_STREAM("robot name:" << robot);
    std::string original_topic = robot + "/fix_base_scan";
    std::string new_topic = robot + "/base_scan";
    sub_ = n_.subscribe(original_topic, 1000, &LaserRepublish::laserCallback,this);
    pub_ = n_.advertise<sensor_msgs::LaserScan>(new_topic, 1000);
  }
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
      sensor_msgs::LaserScan scan = (*msg);
//      scan.header.seq = msg->header.seq;
//      scan.header.stamp.sec = msg->header.stamp.sec;
//      scan.header.stamp.nsec = msg->header.stamp.nsec;
//      scan.header.frame_id = msg->header.frame_id;
      scan.header.frame_id.erase(0,1); // Remove leading '/'

//      scan.angle_min = msg->angle_min;
//      scan.angle_max = msg->angle_max;
//      scan.angle_increment = msg->angle_increment;

//      float32 angle_max
//      float32 angle_increment
//      float32 time_increment
//      float32 scan_time
//      float32 range_min
//      float32 range_max
//      float32[] ranges
//      float32[] intensities

      // Publish the message
      pub_.publish(scan);

   }
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "odo_republish");

  LaserRepublish LaserRepublish;

  ros::spin();

  return 0;
}
