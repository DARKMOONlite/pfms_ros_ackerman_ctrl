#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
//#include <pipes.h>
#include "ros/ros.h"
#include <thread>
#include <mutex>

//! All the messages we need are here
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"

//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    geometry_msgs::Point location;
    //! distance to goal
    double distance;
    //! time to goal
    double time;
};


class Controller: public ControllerInterface
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller();

  //See controllerinterface.h for more information

  /**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  virtual bool calcNewGoal() = 0;

  //ControllerInterface functions (all doxygen comments in the files)
  bool setGoal(geometry_msgs::Point goal);

  pfms::PlatformType getPlatformType(void);

  bool setTolerance(double tolerance);

  double distanceTravelled(void);

  double timeInMotion(void);

  double distanceToGoal(void);

  double timeToGoal(void);

  nav_msgs::Odometry getOdometry(void);

protected:

  /**
   * Checks if the goal has been reached.
   *
   * Update own odometry before calling!
   * @return true if the goal is reached
   */
  bool goalReached();

  //Pipes has been removed, we now use publishers and subscribers
//  Pipes* odo_pipesPtr_; //!< The pipe to recieve odometry
//  Pipes* cmd_pipesPtr_; //!< The pipe to send commands

  nav_msgs::Odometry odo_;//!< The current pose/velocity of platform
  std::mutex odoMtx_; //!< Mutex associated with odometry

  //stats
  GoalStats goal_;
  pfms::PlatformType type_;

  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run
  double tolerance_; //!< Radius of tolerance
  long unsigned int cmd_pipe_seq_; //!<The sequence number of the command

  ros::Publisher viz_pub_;//! Visualisation Marker publisher
  ros::Subscriber sub1_,sub2_,sub3_,sub4_;  // Few subscribers
  ros::ServiceServer service_; // Incoming service

};

#endif // CONTROLLER_H
