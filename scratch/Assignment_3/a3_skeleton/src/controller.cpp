#include "controller.h"

#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>
#include <utility>


using pfms::geometry_msgs::Goal;
using std::vector;
using std::deque;


Controller::Controller(){
    //Just initialize the odometry to 0;
odometry_ = {.seq=0,.x=0,.y=0,.yaw=0,.vx=0,.vy=0,}; // Get base odometry Values for later comparison
start_odometry_ = {.seq=0,.x=0,.y=0,.yaw=0,.vx=0,.vy=0,}; // These have to be like this for the Unit tests. if you run getOdometry it doesn't work.
Start_Time_ = std::chrono::steady_clock::now();
current_goal=0;

status_ = pfms::IDLE;
Time_Travelled_ = 0;
Distance_Travelled_ = 0;
}






bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals){

Goal MockGoal;
for(int i = 0; i<goals.size(); i++){
    
    MockGoal.point = goals.at(i);
    MockGoal.seq = i;
    goals_.push_back(MockGoal);
    

}
Cur_Goal = goals_.at(0);
return(true);
}

double Controller::distanceTravelled(void){
    //std::cout << "Waiting at cv" << std::endl;
    std::unique_lock<std::mutex> lck(thread_mtx_);
    // thread_cv_.wait(lck, [&](){return ready_==true;});
    
    thread_cv_.wait_for(lck,std::chrono::seconds(1),[&](){return ready_==true;});


    //std::cout << "Distance Traveled: " <<Distance_Travelled_ << std::endl;
    ready_ = false;
    lck.unlock();
    return(Distance_Travelled_);}

double Controller::timeTravelled(void){return(Time_Travelled_);}
// Just return the platform type
pfms::PlatformType Controller::getPlatformType(void){return(platform_);}

pfms::PlatformStatus Controller::status(void){
    std::unique_lock<std::mutex> lck(thread_mtx_);
    //std::cout << "controller Status function called " << status_ << std::endl;
    return(status_);
    }

void Controller::getOdometry(const nav_msgs::OdometryConstPtr &msg) {

    nav_msgs::Odometry odo = *msg;
    // ROS_INFO_STREAM("New Odometry Values");  
    

    odometry_ = ConvertOdo2pfms(odo);
        
    estimatedOdometry_ = odometry_;
      if(odo.header.seq%800==0){
        ROS_DEBUG_STREAM("Odometry x: " << odometry_.x << " y: " << odometry_.y << " yaw: " << odometry_.yaw);
      }
    // sendCMD(0,0.2,0.2);
    
}


void Controller::getLaserScan(const sensor_msgs::LaserScanConstPtr& msg){
auto odo = odometry_;  
  ROS_DEBUG_STREAM("Receiving laser scan");
  if(laserProcessingPtr_==nullptr){
    laserProcessingPtr_ = new LaserProcessing(*msg);
  }
  else {
    laserProcessingPtr_->newScan(*msg);
  }
  std::vector<geometry_msgs::Point> pts;
  std::vector<geometry_msgs::Point> apts; //Adjusted based on vehicle odo
  //pts = laserProcessingPtr_->detectPoints();
  
   pts = laserProcessingPtr_->detectAllPoints();
   nearest_cone_ = laserProcessingPtr_->detectPositionClosest(180);
  std::vector<geometry_msgs::Point> newpts;
 
//  ROS_DEBUG_STREAM("Calculating Distance");

  for(int i=0; i<pts.size(); i++){
    geometry_msgs::Point Point ;
    double dist = sqrt(pow(pts.at(i).x,2)+pow(pts.at(i).y,2));
    double alpha = atan2(pts.at(i).y,pts.at(i).x);
    double gamma = alpha+odo.yaw;
    
    Point.x = dist*cos(gamma)+odo.x;
    Point.y = dist*sin(gamma)+odo.y;
   Point.z = pts.at(i).z;
    apts.push_back(Point);
  }

  // auto p = laserProcessingPtr_->detectPositionClosest();
  // // std::cout << "Closest Position: " << p.x << " " << p.y << std::endl;

  //   std::cout <<  pts.at(0).x << " " << pts.at(0).y << std::endl;
  //   std::cout << "dist+ alpha " << sqrt(pow(pts.at(0).x,2)+pow(pts.at(0).y,2)) << " " <<atan2(pts.at(0).y,pts.at(0).x) << std::endl;
  //   std::cout << "Odometry:" << odometry_.x << " " <<odometry_.y << " yaw:" <<odometry_.yaw << std::endl;
   
  // for(int i = 0; i< 10; i++){
  //   auto test = laserProcessingPtr_->detectPoints(i*36);
  //   std::cout << test.size() << " angle: " << i*36 <<  std::endl;
  // }

  bool TooClose = false;
  

  while(apts.size()>0){
    TooClose = false;
    //if the value is too similar to another already known cone
    for(int j = 0; j < knownCones.size(); j++){
      if(abs(apts.front().x-knownCones[j].x)< cone_toll_ && abs(apts.front().y-knownCones[j].y) < cone_toll_ ){
        TooClose= true;
      }

    }
    if(!TooClose){
      knownCones.push_back(apts.front());
      newpts.push_back(apts.front());
    }
    apts.erase(apts.begin());


  }
  // newpts.push_back(apts.front());
  // std::cout<< "Adjusted: " <<  newpts.at(0).x << " " << newpts.at(0).y;

Visualise(newpts,visualization_msgs::Marker::CYLINDER);

// Visualise(newpts,visualization_msgs::Marker::CYLINDER);

}





// Sets the tolerance of the vehicle
bool Controller::setTolerance(double tolerance){Tolerance_ = tolerance;
return(1);
}

void Controller::getSonar(const sensor_msgs::RangeConstPtr &msg){

sensor_msgs::Range sonar = *msg;

 ROS_INFO_STREAM("Sonar Dist: " << sonar.range );
if(sonar.range < sonar.max_range && sonar.range > sonar.min_range && !isinf(sonar.range)){
  
  double dist = sqrt(pow(nearest_cone_.x,2)+pow(nearest_cone_.y,2));
    ROS_WARN_STREAM("Nearest Cone Dist: " <<dist << " x: " << nearest_cone_.x << " y: " << nearest_cone_.y);
    
    if(abs(dist-sonar.range)<0.5){ // if sonar's detected object is not as close as the nearest
      ROS_INFO_STREAM("Cone Detected");
    } 
    else{
    ROS_WARN_STREAM("VEHICLE CLOSE TO OBJECT");
    ROS_WARN_STREAM("STOPPING VEHICLE");
    Running_ = false;
    }
    
  
}
}

void Controller::Visualise(std::vector<geometry_msgs::Point> points, int shape){


  visualization_msgs::MarkerArray Shapes;
  std_msgs::ColorRGBA colour;
        colour.a=0.5;//a is alpha - transparency 0.5 is 50%;
        colour.r=0;
        colour.g=1.0;
        colour.b=0;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration(10000);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color = colour;


  for(int i = 0; i < points.size()  ;i++){
    marker.pose.position.x = points.at(i).x;
    marker.pose.position.y = points.at(i).y;
    marker.pose.position.z = points.at(i).z;
    switch(shape){
      case visualization_msgs::Marker::CYLINDER: 
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;

        marker.ns = "cones";
        marker.id = marker_counter++;
      break;
      case visualization_msgs::Marker::CUBE:
        marker.ns = "road";
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.id = marker_counter++;
      break;  

      default:
      break;
    }
  Shapes.markers.push_back(marker);


  }
  if(Shapes.markers.size() != 0){
    ROS_INFO_STREAM("Visualising Shapes, Volume : " << Shapes.markers.size());
    
    viz_pub_.publish(Shapes);
  }

}

 bool Controller::StartVehicle(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){

   ROS_INFO_STREAM("Vehicle Control Called");
   Running_ = req.data;


  //This needs to return in the vehicle can see anything, 
   resp.success = true;
  // this can be anything 
   resp.message = "Test";
   
  return(1);

 }


//Returns the distance traveled in comparison to the start_odometry_


bool Controller::withinTolerance(){

  if(std::abs(x_distance_) > Tolerance_|| std::abs(y_distance_) > Tolerance_){
    return(false);
  }
  else{return(true);}
}


bool Controller::GoalWithinCones(pfms::geometry_msgs::Goal Goal){

std::vector<geometry_msgs::Point> CloseCones = laserProcessingPtr_->detectPoints(90);
if(CloseCones.empty()|| CloseCones.size()<2){
  std::vector<geometry_msgs::Point> CloseCones = laserProcessingPtr_->detectPoints(170);
  ROS_WARN_STREAM("Warning Not Enough Cones Detected to determine whether goal is inside cones");
}

return(WithinTriangle(Goal,CloseCones,odometry_));

}

bool Controller::WithinTriangle(pfms::geometry_msgs::Goal Goal,std::vector<geometry_msgs::Point> Cones,pfms::nav_msgs::Odometry location){
std::pair<geometry_msgs::Point,double> Close1;
Close1.second = std::numeric_limits<double>::max();
std::pair<geometry_msgs::Point,double> Close2 = Close1;

//This for loop finds the closest 2 points to us

for(int i=0; i<Cones.size(); i++){
  double dist = sqrt(pow(Cones.at(i).x,2) + pow(Cones.at(i).y,2));
  if(dist<Close1.second){
    Close2 = Close1; //Shift previously closest value to 2nd closest
    Close1.second=dist;
    Close1.first = Cones.at(i);
  }
}
ROS_INFO_STREAM("Cone1: x: " << Close1.first.x<<" y: " <<Close1.first.y);
ROS_INFO_STREAM("Cone1: x: " << Close2.first.x<<" y: " <<Close2.first.y);
double alpha1,alpha2;
alpha1 = atan2(Close1.first.y,Close1.first.x)+odometry_.yaw;
alpha2 = atan2(Close2.first.y,Close2.first.x)+odometry_.yaw;

double goalangle = atan2(Goal.point.y-location.y,Goal.point.x-location.x);
ROS_INFO_STREAM("Angle to Cones: "<< alpha1 << ", "<< alpha2 );
ROS_INFO_STREAM("Angle to Goal: " <<goalangle);

if((goalangle>alpha1 && goalangle<alpha2)||(goalangle>alpha2 && goalangle<alpha1)){
  return(true);
}
  return(false);



}
pfms::geometry_msgs::Goal Controller::ConvertGoal2pfms(geometry_msgs::Pose goal){
  pfms::geometry_msgs::Goal ConvertedGoal;
  ConvertedGoal.point.x = goal.position.x;
  ConvertedGoal.point.y = goal.position.y;
  return(ConvertedGoal);
}

geometry_msgs::Pose Controller::ConvertGoalFpfms(pfms::geometry_msgs::Goal Goal){
  geometry_msgs::Pose ConvertedGoal;
  ConvertedGoal.position.x = Goal.point.x;
  ConvertedGoal.position.y = Goal.point.y;
  ConvertedGoal.position.z = 0;
  return(ConvertedGoal);
}

pfms::nav_msgs::Odometry Controller::ConvertOdo2pfms(nav_msgs::Odometry odo){
  pfms::nav_msgs::Odometry ConvertedOdo;
    
   ConvertedOdo.x = odo.pose.pose.position.x;
    ConvertedOdo.y = odo.pose.pose.position.y;
    double w = odo.pose.pose.orientation.w, x = odo.pose.pose.orientation.x, y = odo.pose.pose.orientation.y, z = odo.pose.pose.orientation.z;
    
  // ROS_INFO_STREAM("twist: " << w << " " << x << " " << y << " " << z);
     
    // double yaw = atan2(2*(y*z+w*x),(pow(w,2)-pow(x,2)-pow(y,2)+pow(z,2)));
    double roll = atan2(2.0*(x*y + w*z), (pow(w,2)+pow(x,2)-pow(y,2)-pow(z,2)));
    // double pitch = asin(-2*(x*z-w*y));
    // ROS_INFO_STREAM("Yaw: " << yaw << " roll: " << roll << " pitch: " << pitch);
    ConvertedOdo.yaw = roll;
    ConvertedOdo.vx = odo.twist.twist.linear.x;
    ConvertedOdo.vy = odo.twist.twist.linear.y;
    ConvertedOdo.seq = odo.header.seq;
    
    return(ConvertedOdo);
  
}

void Controller::OverrideRunningCheck(bool value){
  Running_ = value;
}
bool Controller::ReadRunningCheck(){
  return(Running_);
}