#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include <cmath>

class Ackerman: public Controller
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ackerman();
    /**
     * @brief will move the Audi to the goal
     * 
     * @return true if vehicle can reach the goal
     * @return false if vehicle cannot
     */
  bool reachGoal();
    /**
     * @brief Set the Goal object to the internal vector of goals
     * 
     * @param goal 
     * @return true if goal can be reached
     * @return false if goal cannot
     */
  bool setGoal(pfms::geometry_msgs::Point goal);
  /**
   * @brief does calculations for internal values to determine if the goal can be reached. somewhat superceded by checkOriginToDestination
   * 
   * @param goal the final goal location
   * @param odometry the starting value to check from
   * @return true 
   * @return false 
   */
  bool AngleToGoal(pfms::geometry_msgs::Point goal,pfms::nav_msgs::Odometry odometry);
  /**
   * @brief Calls AngleToGoal and returns the distance ti the goal
   * 
   * @return double distance (m)
   */
  double distanceToGoal();
/**
 * @brief Returns the time necessary to reach the goal
 * 
 * @return double time (s)
 */
  double timeToGoal();
  /**
   * @brief Just a basic copy of the snippet. used for debugging
   * 
   * @param repeats number of times to repeat the command
   * @param brake  (n/m) from 0 to 8000 
   * @param throttle @0.1 speed = 2.91m/s
   * @param steering from -pi/2 to pi/2
   */
  void Basic(unsigned int repeats, double brake, double throttle, double steering);
  /**
   * @brief Takes an odometry and determines if a @param goal can be reached from it
   * 
   * @param origin The start location 
   * @param goal the end position
   * @param distance the distance to the goal 
   * @param time  The time to the goal
   * @param estimatedGoalPose  The estimated final odometry position
   * @return true if goal can be reached
   * @return false if goal can't
   */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose);
    

protected:
  const double Length_ = 2.65;
  const double Steering_Ratio = 17.3;
  const double Lock_to_Lock_Revs = 3.2;
  const double Max_Steer_Angle = (M_PI*3.2/17.3);
  double radialDistance_;
  double delta_;
  Pipes* goal_pipe;
  

  

};



#endif // ACKERMAN_H
