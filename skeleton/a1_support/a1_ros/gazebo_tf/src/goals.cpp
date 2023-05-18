#include "ros/ros.h"
#include <string>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>

#include "pipes.h"

using pfms::geometry_msgs::Goal;

class Goals
{
public:
    Goals(ros::NodeHandle nh) :
        nh_(nh),marker_counter_(0),seq_(0)
    {
        viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);
        pipes = new Pipes(pipes::Type::CONSUMER, 5,"/goal_buffer",false);
    }


    ~Goals(){
        delete pipes;
    }

    void run(void){

        ros::Rate rate_limiter(20.0);

        while (ros::ok()){

          Goal goal;
          if(pipes->readCommand(goal)){
              ROS_INFO_STREAM( "seq x/y " <<  goal.seq << " "
                               << goal.point.x << " " << goal.point.y );

              if(goal.seq!=seq_){
                marker_array_.markers.clear();
              }

              addMarker(goal.point.x, goal.point.y, static_cast<unsigned int>(goal.seq));
              //We publish the marker array
              viz_pub_.publish(marker_array_);
              ros::spinOnce();
              seq_=static_cast<unsigned int>(goal.seq);
          }
          else {
              ROS_WARN("No Goal message received in 5 seconds");
           }

          rate_limiter.sleep();
        }
    }


void addMarker(double x, double y, unsigned int seq){
     visualization_msgs::Marker marker;


     //We need to set the frame
     // Set the frame ID and time stamp.
     marker.header.frame_id = "world";
     marker.header.seq = seq;
     marker.header.stamp = ros::Time::now();


     //We set lifetime (it will dissapear in this many seconds)
     marker.lifetime = ros::Duration(200.0);
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "goals";
     marker.id = marker_counter_++;

     // The marker type, we use a cylinder in this example
     marker.type = visualization_msgs::Marker::CYLINDER;

     // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
     marker.action = visualization_msgs::Marker::ADD;

     //As an example, we are setting it
     marker.pose.position.x = x;
     marker.pose.position.y = y;
     marker.pose.position.z = 0;

     //Orientation, can we orientate it?
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;


     // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
     marker.scale.x = 0.5;
     marker.scale.y = 0.5;
     marker.scale.z = 5.0;

     //Alpha is stransparency (50% transparent)
     marker.color.a = 0.5f;

     //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
     marker.color.r = 0.0;
     marker.color.g = 1.0;//static_cast<float>(177.0/255.0);
     marker.color.b = 0.0;

    //We push the marker back on our array of markers
    marker_array_.markers.push_back(marker);

}


private:
    ros::NodeHandle nh_;
    ros::Publisher viz_pub_;
    int marker_counter_;
    unsigned int seq_;
    visualization_msgs::MarkerArray marker_array_;
    Pipes* pipes;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "goals");

  ros::NodeHandle nh;

  std::shared_ptr<Goals> goals(new Goals(nh));

  std::thread run_thread(&Goals::run,goals);

  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  run_thread.join();

  return 0;
}
