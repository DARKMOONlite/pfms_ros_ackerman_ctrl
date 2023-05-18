#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include "pipes.h"
#include <cmath>
#include <iostream>
class Quadcopter: public Controller
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter();
    /**
     * @brief will move the Quadcopter to the goal
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
     * @brief Returns the distance to the latest goal in the goal vector
     * 
     * @return double the distance
     */
    double distanceToGoal();
    /**
     * @brief Returns the time to the latest goal in the goal vector
     * 
     * @return double 
     */
    double timeToGoal();

    
    /**
     * @brief  Just a basic copy of the snippet. used for debugging
     * 
     * @param repeats number of times to repeat movement
     * @param turnl_r turn
     * @param movel_r move left or right
     * @param moveu_d move up or down (not needed in this task)
     * @param movef_b move forward or backward
     */
    void Basic(unsigned int repeats, double turnl_r, double movel_r, double moveu_d, double movef_b );

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
  
private: 
/**
 * @brief A pointer to a pipe to transmit commands to ros
 * 
 */
 Pipes* goal_pipe;
};

#endif // QUADCOPTER_H
