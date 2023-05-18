#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include "visualization_msgs/MarkerArray.h"
#include <atomic>
//! All the services .. RequesteGoal is part of project_setup, not a standard service
#include "project_setup/RequestGoal.h"

#include "laserprocessing.h"

//! UAV drone platform controller
class Quadcopter: public Controller
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter(ros::NodeHandle nh);

  ~Quadcopter();

  bool reachGoal(void);

  /**
   * Calculates the angle needed for the quadcopter to reach a goal.
   * @return Always true - quadcopter has no unreachable goals.
   */
  bool calcNewGoal(void);

  bool checkOriginToDestination(nav_msgs::Odometry origin, geometry_msgs::Point goal,
                                 double& distance, double& time,
                                 nav_msgs::Odometry& estimatedGoalPose);

  /*! @brief seperate thread.
  *
  *  The main processing thread that will run continously and utilise the data
  *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
  */
  void seperateThread();

  /*! @brief Request Goals service callback
   *
   *  @param req The requested goals.
   *  @param res The responce (should have goals reachable at point of call)
   *
   *  @return bool - Will return true to indicate the request sucseeded
   */
    bool requestGoals(project_setup::RequestGoal::Request  &req,
             project_setup::RequestGoal::Response &res);


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
    *  @param geometry_msgs::Pose pose - only the x,y position will be used
    *  @param std_msgs::ColorRGBA color - the color (including transparency) that will be used
    */
  visualization_msgs::MarkerArray produceMarkerArray(geometry_msgs::Pose pose, std_msgs::ColorRGBA color);


  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  //! Angle required for quadcopter to have a straight shot at the goal
  double target_angle_ = 0;

  const double TARGET_SPEED;

  ros::NodeHandle nh_;//Node handle for communication

  std::atomic<bool> goalReceived_; //! Atomic bool to indicate if we have a goal

  LaserProcessing* laserProcessingPtr_;
};

#endif // QUADCOPTER_H
