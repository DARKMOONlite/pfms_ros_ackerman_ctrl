#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

//! All the messages we need are here
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/MarkerArray.h"

//! All the services .. FaceGoal is part of project_setup, not a standard service
#include "project_setup/FaceGoal.h"

#include <atomic>

//! The class we have developed included here in our node
#include "grid_processing.h"


class PfmsSample{

public:
  /*! @brief PfmsSample constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    PfmsSample(ros::NodeHandle nh);

  /*! @brief PfmsSample destructor.
   *
   *  Will tear down the object
   */
    ~PfmsSample();


  /*! @brief Face Goal service callback
   *
   *  @param req The requested goal.
   *  @param res The responce
   *
   *  @return bool - Will return true to indicate the request sucseeded
   */
    bool faceGoal(project_setup::FaceGoal::Request  &req,
             project_setup::FaceGoal::Response &res);


  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /*! @brief OccupancyGrid Callback
    *
    *  @param sensor_msgs::ImageConstPtr - The imageconst message
    *  @note This function and the declaration are ROS specific
    */
     void occupancyGridCallback(const nav_msgs::OccupancyGridPtr & msg);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */
    void seperateThread(); 


private:
    /*! @brief Creates a MarkerArray by adding a single SPHERE marker to it, based on (pose)x,y position
     * of pose supplied and color supplied
      *
      *  @param geometry_msgs::Pose pose - only the x,y position will be used
      *  @param std_msgs::ColorRGBA color - the color (including transparency) that will be used
      */
    visualization_msgs::MarkerArray produceMarkerArray(geometry_msgs::Pose pose, std_msgs::ColorRGBA color);


    ros::NodeHandle nh_;
    ros::Publisher viz_pub_;//! Visualisation Marker publisher

    ros::Subscriber sub1_,sub2_,sub3_;

    ros::ServiceServer service_;

    int count_;//! A counter to allow executing items on N iterations

    //! Question: Is theer a better way to store data, instead of a structure?
    //! refer to Tutorial 7 exercise 3
    struct PoseDataBuffer
    {
      //! Question: Given these elements come in two's (pose and time)
      //! Is there a better type of STL container rather than two seperate deques?
        geometry_msgs::Pose pose;
        std::mutex mtx;
    };
    PoseDataBuffer poseDataBuffer_;//! Container for pose data

    struct OgMapBuffer
    {
        nav_msgs::OccupancyGrid grid;
        std::mutex mtx;
    };

    OgMapBuffer ogMapBuffer_;//! Container for image data

    PoseDataBuffer goalPoseBuffer_; //! We store the goal pose (as it is provided in service and we want to reuse it later)
    std::atomic<bool> goalReceived_; //! Atomic bool to indicate if we have a goal

};

