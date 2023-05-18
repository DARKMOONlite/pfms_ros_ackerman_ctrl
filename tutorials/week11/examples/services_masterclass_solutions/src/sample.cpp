
#include "sample.h"


/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser and Grid)
 * - Respond to an incoming service call
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
  //We can transform either way (global to local) or (local to global)
  //Here we go global to local

  local.x = req.pose.x-pose.position.x;
  local.y = req.pose.y-pose.position.y;


  ROS_INFO_STREAM("Checking x,y=" << local.x << "," << local.y );

  geometry_msgs::Point zero;
  zero.x=0;
  zero.y=0;
  //We store the result of checking connectivity
  //in success field of the responce (if you look at the service message, the success is just a boolean)
  res.success = gridProcessing.checkConnectivity(zero,local);

  if(res.success){
    ROS_INFO("Goal can be reached");
  }
  else{
    ROS_WARN("Goal CAN NOT be reached");
  }

  goalReceived_=true;

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

        std::unique_lock<std::mutex> lck2 (goalPoseBuffer_.mtx);
        geometry_msgs::Pose goalPose = goalPoseBuffer_.pose;
        lck2.unlock();

        geometry_msgs::Point local;
        local.x = goalPose.position.x-robotPose.position.x;
        local.y = goalPose.position.y-robotPose.position.y;

        geometry_msgs::Point zero;
        zero.x=0;zero.y=0;

        //Let's get the Grid
        std::unique_lock<std::mutex> lck3 (ogMapBuffer_.mtx);
        nav_msgs::OccupancyGrid grid = ogMapBuffer_.grid;
        GridProcessing gridProcessing(grid);
        lck3.unlock();

        ROS_INFO_STREAM("Checking x,y=" << local.x << "," << local.y );

        // Why would a goal reachability change?
        // It could have been in uknown space first and then free OR
        // It could have been behind a wall and your robot now has line of sight to it!
        bool reachable = gridProcessing.checkConnectivity(zero,local);

        //Let's send a marker with color (green for reachable, red for now)
        std_msgs::ColorRGBA color;
        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        // the colors r,g,b are floats 0 - 1.

        if(reachable){
          ROS_INFO("Goal can be reached");
          color.r=0;
          color.g=1.0;
          color.b=0;
        }
        else {
          ROS_WARN("Goal CAN NOT be reached");
          color.r=1.0;
          color.g=0;
          color.b=0;
        }

        //Let's also publish the marker here
        visualization_msgs::MarkerArray marker_array = produceMarkerArray(goalPose,color);
        viz_pub_.publish(marker_array);


      }

      rate_limiter.sleep();

    }
}


visualization_msgs::MarkerArray PfmsSample::produceMarkerArray(geometry_msgs::Pose pose,std_msgs::ColorRGBA color){

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
  marker.pose.position.z = 0.0;

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
