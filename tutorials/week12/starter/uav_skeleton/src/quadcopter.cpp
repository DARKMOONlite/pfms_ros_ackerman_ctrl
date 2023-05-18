#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

using std::cout;
using std::endl;

Quadcopter::Quadcopter(ros::NodeHandle nh) :
  nh_(nh),
  TARGET_SPEED(0.4)
{
  // We need to setup the connections to the topics and service required here

  //Subscribing to odometry
  sub1_ = nh_.subscribe("uav_odom", 1000, &Quadcopter::odomCallback,this);
  //Subscribing to laser
  sub2_ = nh_.subscribe("drone/laser/scan", 10, &Quadcopter::laserCallback,this);
  //Subscribing to ranger
  sub3_ = nh_.subscribe("drone/sonar/range", 1, &Quadcopter::rangeCallback,this);
  //Subscribing to clicked points
  sub4_ = nh_.subscribe("clicked_point", 1, &Quadcopter::pointStampedCallback,this);


  //Publishing markers
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

  // Below is how to get parameters from command line, on command line they need to be _param:=value
  // For example _example:=0.1
  // ROS will obtain the configuration from command line, or assign a default value 0.1
  ros::NodeHandle pn("~");
  double example;
  pn.param<double>("example", example, 0.1);
  ROS_INFO_STREAM("example:" << example);

  goalReceived_=false;// We will use this atomic bool to let us know when we have new data

  //Allowing an incoming service on /request_goal
  service_ = nh_.advertiseService("request_goals", &Quadcopter::requestGoals,this);

  type_ = pfms::PlatformType::QUADCOPTER; //Type is quadcopter
  tolerance_=0.5;//We set tolerance to be default of 0.5
};

// We delete the pipes here specifically, which forces them to close before the object is terminated
Quadcopter::~Quadcopter(){


}

bool Quadcopter::checkOriginToDestination(nav_msgs::Odometry origin, geometry_msgs::Point goal,
                              double& distance, double& time,
                              nav_msgs::Odometry& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.pose.pose.position.x;
    double dy = goal.y - origin.pose.pose.position.y;

    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;

    // The estimated goal pose would be the goal at the angle we had at the origin
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.pose.pose.position.x = goal.x;
    estimatedGoalPose.pose.pose.position.y = goal.y;
    estimatedGoalPose.pose.pose.orientation = origin.pose.pose.orientation;
    estimatedGoalPose.twist.twist.linear.x = 0;
    estimatedGoalPose.twist.twist.linear.y = 0;

    return true;
}
bool Quadcopter::calcNewGoal(void) {

    getOdometry();//This will update internal copy of odometry, as well as return value if needed.

    nav_msgs::Odometry est_final_pos;

    if (!checkOriginToDestination(odo_, goal_.location, goal_.distance, goal_.time, est_final_pos))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x - odo_.pose.pose.position.x;
    double dy = goal_.location.y - odo_.pose.pose.position.y;
    target_angle_ = std::atan2(dy, dx);

    return true;
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {
 //! What do we need to do to control the Quadcopter?
 //! What do we advertise, and what do we publish?
//    pfms::commands::UAV cmd = {
//        cmd_pipe_seq_++,
//        turn_l_r,
//        move_l_r,
//        move_u_d,
//        move_f_b,
//    };
//    cmd_pipesPtr_->writeCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));//Small delay to ensure message sent
}

bool Quadcopter::reachGoal(void) {
    calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in
    auto start_odo = odo_;
    auto start_time = std::chrono::system_clock::now();

    //Run below loop until we reach goal
    while (!goalReached()) {

        // Get relative target angle
        double yaw = tf::getYaw(odo_.pose.pose.orientation);
        double theta = yaw - target_angle_;

        // Move at `speed` in target direction
        double dx = TARGET_SPEED * std::cos(theta);
        double dy = TARGET_SPEED * std::sin(theta);

        sendCmd(0, -dy, 0, dx);

        calcNewGoal(); //get odometry and update target angle of control

        // Update distance travelled
        auto finish_odo = odo_;
        double mx = finish_odo.pose.pose.position.x - start_odo.pose.pose.position.x;
        double my = finish_odo.pose.pose.position.y - start_odo.pose.pose.position.y;
        distance_travelled_ += std::hypot(mx, my);
        start_odo=odo_;
        //std::cout <<  distance_travelled_ << " , " << goal_.distance << std::endl;
    }

    // Stop thq quadcopter immediately
    sendCmd(0, 0, 0, 0);

    // Update time taken
    auto finish_time = std::chrono::system_clock::now();
    //std::chrono::seconds is integer for some reason, thus duration<double>
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(finish_time - start_time);
    time_travelled_ += time_taken.count();

    calcNewGoal(); //get odometry and update distance to goal

    return true;
}


// A callback for odometry
void Quadcopter::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // We store a copy of the odometry and lock a mutex when updating
    std::unique_lock<std::mutex> lck (odoMtx_);
    odo_ = *msg;
}

void Quadcopter::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

}

void Quadcopter::rangeCallback(const sensor_msgs::RangeConstPtr &msg)
{

}

void Quadcopter::pointStampedCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  geometry_msgs::Point pt;
  pt.x = msg->point.x;
  pt.y = msg->point.y;
  pt.z = msg->point.z;
  setGoal(pt);
}



void Quadcopter::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    //! What rate shouls we run this at? What does this limiter do?
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) {

      //! @todo Ex02 : Can we check here every 1s wether there is someone below us
      //!
      //! To do this check, what information do we need?
      //!
      std::unique_lock<std::mutex> lck (odoMtx_);
      nav_msgs::Odometry odo = odo_;
      lck.unlock();

//        //Below is how to get yaw ... if your still wondering
//        //double robotYaw = tf::getYaw(robotPose.orientation);

//        std::unique_lock<std::mutex> lck2 (goalPoseBuffer_.mtx);
//        geometry_msgs::Pose goalPose = goalPoseBuffer_.pose;
//        lck2.unlock();

//        geometry_msgs::Point local;
//        local.x = goalPose.position.x-robotPose.position.x;
//        local.y = goalPose.position.y-robotPose.position.y;

//        geometry_msgs::Point zero;
//        zero.x=0;zero.y=0;


//        //Let's send a marker with color (green for reachable, red for now)
//        std_msgs::ColorRGBA color;
//        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
//        // the colors r,g,b are floats 0 - 1.

//        if(reachable){
//          ROS_INFO("Goal can be reached");
//          color.r=0;
//          color.g=1.0;
//          color.b=0;
//        }
//        else {
//          ROS_WARN("Goal CAN NOT be reached");
//          color.r=1.0;
//          color.g=0;
//          color.b=0;
//        }

//        //Let's also publish the marker here
//        visualization_msgs::MarkerArray marker_array = produceMarkerArray(goalPose,color);
//        viz_pub_.publish(marker_array);
//      }

      rate_limiter.sleep();

    }
}

visualization_msgs::MarkerArray Quadcopter::produceMarkerArray(geometry_msgs::Pose pose,std_msgs::ColorRGBA color){

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
  marker.ns = "test";
  marker.id = marker_counter++;

  // The marker type, we use a cylinder in this example
  marker.type = visualization_msgs::Marker::CYLINDER;

  // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = pose.position.z;

  //Orientation, we are not going to orientate it
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


bool Quadcopter::requestGoals(project_setup::RequestGoal::Request  &req,
             project_setup::RequestGoal::Response &res)
{
  //When an incoming call arrives, we can respond to it here
  //Check what is in the service via  rossrv info project_setup/RequestGoal

  for(auto pose : req.pose.poses){
    ROS_INFO_STREAM("Requested:" << pose.position.x << "," << pose.position.y << "," << pose.position.z);
  }


//    ROS_INFO("All Goals can be reached");
//    res.success=true;
//    ROS_WARN("Goals CAN NOT be reached");

  goalReceived_=true;

  return true; //We return true to indicate the service call sucseeded (your responce should indicate a value)
}
