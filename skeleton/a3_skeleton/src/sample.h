#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

//Keep only the headers needed
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

//We include header of anotehr class we are developing
#include "laserprocessing.h"

//! Sample
class Sample
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Sample(ros::NodeHandle nh);

  ~Sample();


  /*! @brief seperate thread.
  *
  *  The main processing thread that will run continously and utilise the data
  *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
  */
  void seperateThread();

  /*! @brief request service callback
   *
   *  @param req The request
   *  @param res The responce
   *
   *  @return bool - Will return true to indicate the request sucseeded
   */
  bool request(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res);

private:
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

  /*! @brief Range Callback
    *
    *  @param sensor_msgs::RangeConstPtr - The range message
    *  @note This function and the declaration are ROS specific
    */
  void rangeCallback(const sensor_msgs::RangeConstPtr& msg);

  /*! @brief PointStamped Callback
    *
    *  @param sensor_msgs::PointStampedConstPtr - The range message
    *  @note This function and the declaration are ROS specific
    */
  void pointStampedCallback(const geometry_msgs::PointStampedConstPtr& msg);


  /*! @brief Creates a MarkerArray by adding a single SPHERE marker to it, based on (pose)x,y position
   * of pose supplied and color supplied
    *
    *  @param geometry_msgs::Point point - location of point
    *  @param std_msgs::ColorRGBA color - the color (including transparency) that will be used
    */
  visualization_msgs::MarkerArray produceMarkerArray(geometry_msgs::Point point,std_msgs::ColorRGBA color);


  /*! @brief Let's publish a velocity command (this one is for UAV)
    *
    *  @param turn_l_r - angular velocity left/right (left positive)
    *  @param move_l_r - linear velocity left/right (left positive)
    *  @param move_u_d - linear velocity up/down (up positive)
    *  @param move_f_b - linear velocity fowrad/back (forward positive)
    */
  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);


  /*! @brief Let's publish a velocity command (this one is for UGV)
    *
    *  @param brake - brake control in [Nm]  0-8000
    *  @param steering - steering wheel angle
    *  @param throttle - thorttle 0-1
    */
  void sendCmd(double brake, double steering, double throttle);

private:

  ros::NodeHandle nh_;//Node handle for communication

  ros::Publisher viz_pub_;//! Visualisation Marker publisher
  ros::Publisher cmd_pub_, takeOff_pub_;//! Velocity and TakeOff (UAV) publisher
  ros::Publisher brake_cmd_pub_, steering_cmd_pub_, throttle_cmd_pub_; //! Brake, Steering and Throttle (UGV) publishers

  ros::Subscriber sub1_,sub2_,sub3_,sub4_;  // Few subscribers
  ros::ServiceServer service_; // Incoming service

  LaserProcessing* laserProcessingPtr_; //! Pointer to Laser Object

  nav_msgs::Odometry odo_;//!< The current pose/velocity of platform
  std::mutex odoMtx_; //!< Mutex associated with odometry

  bool isUGV_;
  bool isFlying_;

};

#endif // SAMPLE_H
