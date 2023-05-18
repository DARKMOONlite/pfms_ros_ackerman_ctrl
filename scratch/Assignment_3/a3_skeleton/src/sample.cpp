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

using std::cout;
using std::endl;

Sample::Sample(ros::NodeHandle nh) :
  nh_(nh),
  laserProcessingPtr_(nullptr),
  isFlying_(false)
{


    // Below is how to get parameters from command line, on command line they need to be _param:=value
    // For example _example:=0.1
    // ROS will obtain the configuration from command line, or assign a default value 0.1
    ros::NodeHandle pn("~");
    double example;
    pn.param<double>("example", example, 0.1);
    ROS_INFO_STREAM("example:" << example);

    pn.param<bool>("is_ugv", isUGV_, true);

    // We need to setup the connections to the topics and service required here

    if(isUGV_){
        ROS_INFO("Controlling UGV");

        //Subscribing to odometry UGV
        sub1_ = nh_.subscribe("ugv_odom",1000, &Sample::odomCallback,this);

        //Subscribing to laser UGV
        sub2_ = nh_.subscribe("orange/laser/scan", 10, &Sample::laserCallback,this);

        //Subscribing to ranger UGV
        sub3_ = nh_.subscribe("orange/sonar/range", 1, &Sample::rangeCallback,this);

        //Publishing commands for control UGV
        brake_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/orange/brake_cmd", 1024,false);
        steering_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/orange/steering_cmd", 1024,false);
        throttle_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/orange/throttle_cmd", 1024,false);

    }
    else {
        ROS_INFO("Controlling UAV");

        //Subscribing to odometry UAV
        sub1_ = nh_.subscribe("uav_odom", 1000, &Sample::odomCallback,this);

        //Subscribing to laser UAV
        sub2_ = nh_.subscribe("drone/laser/scan", 10, &Sample::laserCallback,this);

        //Subscribing to ranger UAV
        sub3_ = nh_.subscribe("drone/sonar/range", 1, &Sample::rangeCallback,this);

        //Publishing command for control UAV
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("drone/cmd_vel", 1024,false);
        takeOff_pub_ = nh_.advertise<std_msgs::Empty>("/drone/takeoff", 1024,false);

    }

    //Publishing markers
    viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

    //Allowing an incoming service on /request_goal (you need to change name depending on project)
    // Function requst will be called
    service_ = nh_.advertiseService("request_goals", &Sample::request,this);
};

// We delete anything that needs removing here specifically
Sample::~Sample(){

    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
}


// A callback for odometry
void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // Example - We store a copy of the odometry and lock a mutex when updating
    std::unique_lock<std::mutex> lck (odoMtx_);
    odo_ = *msg;
}

void Sample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  // Example - create a pointer to laser processing and call a function.
   // Think - how would you use this point to draw on in seperate thread?
  if(laserProcessingPtr_==nullptr){
    laserProcessingPtr_ = new LaserProcessing(*msg);
  }
  else {
    laserProcessingPtr_->newScan(*msg);
  }
  geometry_msgs::Point pt;
  pt = laserProcessingPtr_->detectPositionClosest();
  ROS_INFO_STREAM("Closest pt in laser:" << pt.x << "," << pt.y);

}

void Sample::rangeCallback(const sensor_msgs::RangeConstPtr &msg)
{
    //No example here, depends on your own code

}

void Sample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    //! THINK : What rate shouls we run this at? What does this limiter do?
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {

        std::unique_lock<std::mutex> lck (odoMtx_);
        nav_msgs::Odometry odo = odo_;
        lck.unlock();

        //Below is how to get yaw ... if your still wondering
        //double robotYaw = tf::getYaw(robotPose.orientation);


        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::ColorRGBA color;
        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        color.r=0;
        color.g=1.0;
        color.b=0;

        //Here as a example, we will just add a meter offset from the locatio of the robot
        //to draw a cyliner
        geometry_msgs::Point pt;
        pt = odo_.pose.pose.position;
        pt.x+=1.0;
        pt.y+=1.0;
        pt.z+=1.0;

        //Let's also publish the marker here
        visualization_msgs::MarkerArray marker_array = produceMarkerArray(pt,color);
        viz_pub_.publish(marker_array);

        rate_limiter.sleep();

        //Just an example here, we will send the UGV velocity commands / or the UAV depening on parameter passed
        if(isUGV_){
            ROS_INFO("UGV Acclerate and Turn...");
            sendCmd(0,1.0,0.1);
        }
        else {
            //What shoudl we do if the quadcopter has not taken off?
            if(!isFlying_){
                takeOff_pub_.publish(std_msgs::Empty());
                ROS_INFO("UAV Taking Off...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));//Delay to ensure message sent and quad reaches safe level
                isFlying_ = true;
            }
            else{
                ROS_INFO("UAV Fly left ...");
                sendCmd(0.0,0.4,0.0,0.0);
            }
        }

    }
}

visualization_msgs::MarkerArray Sample::produceMarkerArray(geometry_msgs::Point point,std_msgs::ColorRGBA color){

    visualization_msgs::MarkerArray marker_array;

    int marker_counter=0;
    visualization_msgs::Marker marker;

    //We need to set the frame
    // Set the frame ID and time stamp.
    marker.header.frame_id = "world";
    //single_marker_person.header.stamp = ros::Time();
    marker.header.stamp = ros::Time::now();


    //We set lifetime (it will dissapear in this many seconds)
    marker.lifetime = ros::Duration(5.0);
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "test"; //This is namespace, markers can be in diofferent namespace
    marker.id = marker_counter++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

    // The marker type, we use a cylinder in this example
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;

    //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;


    // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    //Alpha is stransparency (50% transparent)
    marker.color = color;

    //We push the marker back on our array of markers
    marker_array.markers.push_back(marker);

    return marker_array;

}


bool Sample::request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{
    //When an incoming call arrives, we can respond to it here
    //Check what is in the service via  rossrv info project_setup/RequestGoal

    ROS_INFO_STREAM("Requested:" << req.data);

    return true; //We return true to indicate the service call sucseeded (your responce should indicate a value)
}


void Sample::sendCmd(double brake, double steering, double throttle){
    //Remember week09? To send a std_msgs/Float64 data type we need to make the
    std_msgs::Float64 val;
    val.data = brake;
    brake_cmd_pub_.publish(val);
    val.data = steering;
    steering_cmd_pub_.publish(val);
    val.data = throttle;
    throttle_cmd_pub_.publish(val);
}

void Sample::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b){

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = move_f_b;
    twist_msg.linear.y = move_l_r;
    twist_msg.linear.z = move_u_d;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = turn_l_r;
    cmd_pub_.publish(twist_msg);

}
