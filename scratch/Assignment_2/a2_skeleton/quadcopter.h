#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include "pipes.h"
#include <cmath>
#include <iostream>
class Quadcopter: public Controller
{
public:
  /**
   * @brief Construct a new Quadcopter object
   * 
   */
  Quadcopter();



/**
 * @brief Destroy the Quadcopter object
 * 
 */
  ~Quadcopter();


  /**
   * @brief This function creates the thread for the vehicle
   * 
   */
  void run(); 


  /**
   * @brief This is the function that controls the vehicle, it runs within a seperate thread
   * 
   */
  void runThread();
    /**
     * @brief will move the Quadcopter to the goal
     * 
     * @return true if vehicle can reach the goal
     * @return false if vehicle cannot
     */
    bool reachGoal(int Goal_ID);
    
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
  

    //LinkedOdometry DistanceFromTo(/*  */pfms::nav_msgs::Odometry& Odo, Goal goal);
  /**
   * @brief Updates dist and time for mission calculations
   * 
   * @return true 
   * @return false 
   */
  bool Update_DistTime();

private: 
/**
 * @brief A pointer to a pipe to transmit commands to ros
 * 
 * 
 */
 Pipes* goal_pipe;
 /**
  * @brief This value is how fast the quadcopter should move based on how far away it is from the goal
  * 
  */
  float Target_Speed;
  /**
   * @brief This is High Precision speed, thus slow
   * 
   */
  const float HP_Speed = 1;
  /**
   * @brief Low Precision Speed therefore faster that HP
   * 
   */
  const float LP_Speed = 2;
  /**
   * @brief Maximum Speed I want to travel
   * 
   */
  const float MX_Speed = 3;
 /**
  * @brief  This value is used to determine when the Quadcopter should slow down. 
  * Once it is that close to the goal
  * 
  */
  float HighPrecisionRange = 3;
  /**
   * @brief Used to dertemine what speed the quadcopter should fly at, if the goal is more than 
   * this value away then it will use it's top speed
   * 
   */
  float LowPrecisionRange = 10;

 std::thread Quadcopter_Thread;
};

#endif // QUADCOPTER_H
