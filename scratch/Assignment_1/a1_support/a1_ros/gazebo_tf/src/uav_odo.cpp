#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
#include "tf/transform_datatypes.h"
#include "pipes.h"

class UAVOdo
{
public:
UAVOdo(ros::NodeHandle nh) :
    nh_(nh)
{
    sub_ = nh_.subscribe("/uav_odom/", 1000, &UAVOdo::odoCallback,this);
    pipes = new Pipes(7,"/uav_odo_buffer_seg","/uav_odo_buffer_wsem","/uav_odo_buffer_rsem");
    lastMsg_ = ros::Time::now();
}

~UAVOdo(){
    delete pipes;
}

void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    ROS_INFO_STREAM(msg->header.seq);
    pfms::nav_msgs::Odometry odo;
    odo.seq =msg->header.seq;
    odo.x = msg->pose.pose.position.x;
    odo.y = msg->pose.pose.position.y;
    odo.yaw = tf::getYaw(msg->pose.pose.orientation);
    odo.vx = msg->twist.twist.linear.x;
    odo.vy = msg->twist.twist.linear.y;
    bool OK = pipes->writeCommand(odo);
    if(OK){
      lastMsg_ = ros::Time::now();
    }
    else {
      ROS_WARN_STREAM("No UAV Odo requested in 5 seconds");
      delete pipes;
      pipes = new Pipes(7,"/uav_odo_buffer_seg","/uav_odo_buffer_wsem","/uav_odo_buffer_rsem");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      lastMsg_ = ros::Time::now();
    }
}


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    Pipes* pipes;
    ros::Time lastMsg_;
};


/**
 * May need two threads to allow reconnecting if a timeout occurs (no comms)
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_odo");

  ros::NodeHandle nh;

  std::shared_ptr<UAVOdo> uav_odo(new UAVOdo(nh));

  ros::spin();

  ros::shutdown();

  return 0;
}
