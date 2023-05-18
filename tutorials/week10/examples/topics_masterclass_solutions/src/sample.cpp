#include "sample.h"
#include "radialdetectionogmap.h"

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser and OccupancyGrid)
 * - Publishing visualisation markers
 * - The need to exchange data betweena seperate thread of execution and callbacks
 */

using std::cout;
using std::endl;


PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh)
{
    //Subscribing to odometry
    sub1_ = nh_.subscribe("robot_0/odom", 1000, &PfmsSample::odomCallback,this);
    //Subscribing to laser
    sub2_ = nh_.subscribe("robot_0/base_scan", 10, &PfmsSample::laserCallback,this);
    //Subscribing to occupnacy grid
    sub3_ = nh_.subscribe("local_map/local_map", 1, &PfmsSample::occupancyGridCallback,this);


    //Publishing an image ... just to show how
    viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

    // Below is how to get parameters from command line, on command line they need to be _param:=value
    // For example _example:=0.1
    // ROS will obtain the configuration from command line, or assign a default value 0.1
    ros::NodeHandle pn("~");
    double example;
    pn.param<double>("example", example, 0.1);

    //The below get's configurations from parameters server
    double resolution;
    nh_.getParam("/local_map/map_resolution", resolution);

    ROS_INFO_STREAM("example:" << example);

}

PfmsSample::~PfmsSample()
{

}



// A callback for odometry
void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    /**
     * @todo - Ex 1: Obtain a pose (x,y yaw) from nav_msgs/Odometry
     *

     * - On command line type 'rosmsg show nav_msgs/Odometry'
     * - The position and orientation are in two seperate parts of the message
     * - The orinetation is provided as a quaternion
     * - Which angle to we need?
     * - Ros has a 'tf' library with a helper function to get yaw from the quaternion
     * - http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     * - Consider: Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
     * - Where is time of this message stored
     */


    geometry_msgs::Pose pose = msg->pose.pose;

// We can use 2 different verbosty levels, and swicth then ON/OFF dynamically
//    ROS_INFO_STREAM("x: " << msg->pose.pose.position.x
//                << ",  y: " << msg->pose.pose.position.y
//                << ",  yaw: "<< tf::getYaw(msg->pose.pose.orientation));

//    ROS_DEBUG_STREAM("x: " << msg->pose.pose.position.x
//                << ",  y: " << msg->pose.pose.position.y
//                << ",  yaw: "<< tf::getYaw(msg->pose.pose.orientation));

    std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
    robotPose_ = pose; // We copy the pose here
}



void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  /**
   * @todo - Ex 1 : Find the closest point {x,y} to the robot using sensor_msgs::LaserScan
   *
   * On command line type 'rosmsg show sensor_msgs/LaserScan'
   * What are we provided in this message?
   * Do we have the information in this message to find the closest point?
   * What part of the message do we need to iterate over?
   * How do we convert from range data to {x,y} [this is known as polar to cartesian](https://www.mathsisfun.com/polar-cartesian-coordinates.html)
   * Where is time of this message stored?
   * Is the closest point identified the same as the one you see as closest on the stage simulator? Why is this the case?
   */

  float nearest_range = msg->range_max;
  unsigned int nearest_index = 0;
  for (unsigned int i=0; i<msg->ranges.size(); i++) {
      if (msg->ranges.at(i) < nearest_range) {
          nearest_range = msg->ranges[i];
          nearest_index = i;
      }
  }

  float nearest_angle = msg->angle_min + nearest_index * msg->angle_increment;

  std::unique_lock<std::mutex> lck (closestLaserPointMtx_);
  closestLaserPoint_.x = nearest_range * cos(nearest_angle);
  closestLaserPoint_.y = nearest_range * sin(nearest_angle);
  closestLaserPoint_.z = 0;

  ROS_INFO_STREAM("Nearest obstacle in laser: " << closestLaserPoint_.x << ", " << closestLaserPoint_.y);

}

void PfmsSample::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{
    /**
    * The occupnacy grid is built so the centre point of the map
    * (width/2 and height/2) is at the location of the robot.
    *
    * The value of each cell represents cccupancy probability
    * which is in the range [0,100].  Unknown is -1.
    *
    */
      // nav_msgs/OccupancyGrid Message
    // http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html

    /**
     * @todo Ex 2 : Display the value of the cell at [x=-5,y-5] relative to the
     *
     * - Where is metad data for the map stored (MetaData contains: width, height and resolution)
     * - How would we find any arbitrary x,y combining the map origin, the width,height and resolution?
     *  ^ y
     *  |
     *  |
     *  o------> x
     * - Print out the cell values at (x=-5, y=-5) relative to the centre? Is it occupied?
     */

    // The two points that we are querying are here:
    float x=-5.0;
    float y=-5.0;


     // To do the conversion, we need the location of the origin (which is stored in position)
     geometry_msgs::Point point = msg->info.origin.position;
     // We compute the row and column as the distance from the origin divided by resolution
     int row = static_cast<int>((x - point.x)/msg->info.resolution);
     int col = static_cast<int>((y - point.y)/msg->info.resolution);

     if((row >=0) && (col >=0)  && (col<msg->info.width) && (row<msg->info.height)){
          unsigned int index = static_cast<unsigned int>(row * msg->info.width + col);
          ROS_INFO_STREAM("val:" << static_cast<int>(msg->data.at(index)) );
     }
     else{
       ROS_INFO_STREAM("Outside of the OgMap!");
     }


     /**
      * @todo Ex 3 : Find [x,y] of the the closest occupied cell relative to the robot
      *
      * - How should we search for the closest point to make it more effective
      */

     RadialDetectionOgMap rdOgmap(msg->info.height,
                                  msg->info.width,
                                  msg->data);

     //To detect the closest point, we are best to search radially out from the robot centre.
     //The most efficient algorithm for this is know as the Midpoint circle algorithm
     // It returns the index of the closest obstacles point in our row-major array
     int index = rdOgmap.midPointCircleDetectOccupied(msg->info.width/2);

//     ROS_INFO_STREAM("val:" << static_cast<int>(msg->data.at(index)));
     row = index / static_cast<int>(msg->info.width);//This now gives number of rows as it is a row major
     col = index - (row* static_cast<int>(msg->info.width)); //This is number of columns (it's a row major organisation so we sort rows first)

     x = col - static_cast<int>((msg->info.width/2)); // Now convert to coordinates from centre
     y = row - static_cast<int>((msg->info.height/2)); // Now convert to coordinates from centre

     x *= msg->info.resolution; // Finally apply the resolution
     y *= msg->info.resolution; // Finally apply the resolution

     ROS_INFO_STREAM("index: " << index << " position: row=" << row << " col=" << col );
     ROS_INFO_STREAM("position:  x=" << x << " y=" << y );


     std::unique_lock<std::mutex> lck (closestOgMapPointMtx_);
     closestOgMapPoint_.x=x;
     closestOgMapPoint_.y=y;
     closestOgMapPoint_.z=0;
}




visualization_msgs::MarkerArray PfmsSample::createMarkerArray(double x, double y){
  //! Here is an example of publishing a marker (in a marker array)
  //!
  //!

  visualization_msgs::MarkerArray marker_array; // Marker Array that will be published
  int marker_counter=0;
  visualization_msgs::Marker marker;

  //We need to set the frame
  // Set the frame ID and time stamp.
  marker.header.frame_id = "world";
  //single_marker_person.header.stamp = ros::Time();
  marker.header.stamp = ros::Time::now();


  //We set lifetime (it will dissapear in this many seconds)
  marker.lifetime = ros::Duration(2.0);
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "test";
  marker.id = marker_counter++;

  // The marker type, we use a cylinder in this example
  marker.type = visualization_msgs::Marker::CYLINDER;

  // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
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
  marker.color.a = 0.5f;

  //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
  marker.color.r = 1.0;
  marker.color.g = static_cast<float>(177.0/255.0);
  marker.color.b = 0.0;

  //We push the marker back on our array of markers
  marker_array.markers.push_back(marker);

  return marker_array;
}

void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    */

    // The below gets the current Ros Time
    //ros::Time timeSample = ros::Time::now();

    //! What does this rate limiter do?
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) {

      /**
       * @todo Ex 4 : Find and mark the closest point in global coordinates [x,y]
       * using the robot pose and closest point in laser data
       *
       * - We need to combine the information from pose and laser into this thread
       * - We need to combine the information from pose and occupancygrid into a this thread
       * - We use the visualisation marker code below (shall we put it in a function?)
       */

      //WE ARE UNABLE TO SUPPLY SOLUTION (1) with Laser and Robot Pose UNTIL ALL after A2 SUBMISSIONS
      // You will need to combine pose of robot and the closest point from Laser

      // For OgMap the orientation plays no part

      //Let's get pose
      std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
      std::unique_lock<std::mutex> lck2 (closestOgMapPointMtx_);

      double x = robotPose_.position.x + closestOgMapPoint_.x;
      double y = robotPose_.position.y + closestOgMapPoint_.y;

      lck2.unlock();
      lck1.unlock();

//      ROS_INFO_STREAM("robot   :  x=" << robotPose_.position.x << " y=" << robotPose_.position.y );
//      ROS_INFO_STREAM("ogmap   :  x=" << closestOgMapPoint_.x << " y=" << closestOgMapPoint_.y );
//      ROS_INFO_STREAM("position:  x=" << x << " y=" << y );

      visualization_msgs::MarkerArray marker_array = createMarkerArray(x,y);

      //We publish the marker array
      viz_pub_.publish(marker_array);

      rate_limiter.sleep();
    }
}

