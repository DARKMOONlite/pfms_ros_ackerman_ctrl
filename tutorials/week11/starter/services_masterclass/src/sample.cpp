
#include "sample.h"


/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser and Grid)
 * - Respond to an incoming service call
 * - Publishing a visualisation message
 *
 */

PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh)
{
  //Subscribing to odometry
  sub1_ = nh_.subscribe("robot_0/odom", 1000, &PfmsSample::odomCallback,this);
  //Subscribing to laser
  sub2_ = nh_.subscribe("robot_0/base_scan", 10, &PfmsSample::laserCallback,this);
  //Subscribing to occupnacy grid
  sub3_ = nh_.subscribe("local_map/local_map", 1, &PfmsSample::occupancyGridCallback,this);


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

  //Allowing an incoming service on /face_goal
  service_ = nh_.advertiseService("face_goal", &PfmsSample::faceGoal,this);

}

PfmsSample::~PfmsSample()
{

}


bool PfmsSample::faceGoal(project_setup::FaceGoal::Request  &req,
             project_setup::FaceGoal::Response &res)
{
  //When an incoming call arrives, we can respond to it here
  ROS_INFO_STREAM("request: [x,y]=[" << req.pose.x << "," << req.pose.y << "]");

  //We make a copy of the pose request to share it in other threads of processing (Ex5 for instance)
  {
    std::unique_lock<std::mutex> lck (goalPoseBuffer_.mtx);
    goalPoseBuffer_.pose.position.x=req.pose.x;
    goalPoseBuffer_.pose.position.y=req.pose.y;
  }

  //Let's get the Grid
  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  nav_msgs::OccupancyGrid grid = ogMapBuffer_.grid;

  // We now construct gridPorcessing, which takes a grid and internally makes a copy of the grid
  // Which is why we unlock here immediately therafter
  GridProcessing gridProcessing(grid);
  lck.unlock();

  geometry_msgs::Point local;

  std::unique_lock<std::mutex> lck2 (poseDataBuffer_.mtx);
  geometry_msgs::Pose pose = poseDataBuffer_.pose;
  lck2.unlock();

 //! @todo Ex04 : Adjust the code so it returns a `true` in the acknowledgment of the service call if the point
 //! can be reached in a straight line from current robot pose, only traversing free space.
 //! Check the service message via "rossrv info project_setup/FaceGoal"
 //! We have to transform global to local (robot reference frame)

  if(!gridProcessing.checkConnectivity(pose.position,goalPoseBuffer_.pose.position)){
    return false;
  }
  double angle = atan2(goalPoseBuffer_.pose.position.x-pose.position.x,goalPoseBuffer_.pose.position.y-pose.position.y);
  double w = pose.orientation.w; double x = pose.orientation.x; double y = pose.orientation.y; double z = pose.orientation.z;
  double eulerYaw = atan(2*(w*y+x*z)/(pow(w,2)-pow(z,2)-pow(y,2)+pow(z,2)));
  std::cout << "angle: "<< angle<<" eulerAngle: " <<eulerYaw <<std::endl;
  if(abs(angle- eulerYaw )<0.05){
    return true;
  }
  else{
    return false;
  }




   return true; //We return true to indicate the service call sucseeded (your responce should indicate a value)
}

// A callback for odometry
void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // We store a copy of the pose and lock a mutex when updating
    std::unique_lock<std::mutex> lck (poseDataBuffer_.mtx);
    poseDataBuffer_.pose = msg->pose.pose;

}


void PfmsSample::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{

  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;

}

void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

}


void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    //! What rate shoudl we run this at?
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) {

      //! @todo Ex05 : Check wether the requested goal can be reached every 5 seconds.
      //!
      //! To do this check, what information do we need?
      //!
      if(goalReceived_) {
        //Let's check which way we should face
        std::unique_lock<std::mutex> lck (poseDataBuffer_.mtx);
        geometry_msgs::Pose robotPose = poseDataBuffer_.pose;
        lck.unlock();

        //Below is how to get yaw ... if your still wondering
        //double robotYaw = tf::getYaw(robotPose.orientation);

	//What information do we need for this task? How do we obtain it?
	// We should also visualise that the point can be reached (or not) by publishing 
 	// a marker (sphere), colour it green if it can be reached and red otherwise.
	// Have a look at produceMarkerArray


      }

      rate_limiter.sleep();

    }
}

