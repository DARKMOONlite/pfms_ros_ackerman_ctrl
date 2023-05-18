#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include <cmath>

class Ackerman: public Controller
{
public:
  /**
   * @brief Construct a new Ackerman object
   * 
   */
  Ackerman();

/**
 * @brief Destroy the Ackerman object
 * 
 */
  ~Ackerman();


/**
 * @brief Runs Ackerman and creates thread
 * 
 */
  void run();

/**
 * @brief This runs in the thread and controlls the vehicle
 * 
 */
  void runThread();

    /**
     * @brief will move the Audi to the goal
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
    
  /**
   * @brief Updates Distance and time for threading work
   * 
   * @return true 
   * @return false 
   */
  bool Update_DistTime();



protected:
/**
 * @brief Length of Car for ackerman steering
 * 
 */
  const double Length_ = 2.65;
  /**
   * @brief Steering ratio of ackerman
   * 
   */
  const double Steering_Ratio = 17.3;
  /**
   * @brief Revolutions wheel can turn
   * 
   */
  const double Lock_to_Lock_Revs = 3.2;
  /**
   * @brief Maximum steering angle of vehicle
   * 
   */
  const double Max_Steer_Angle = (M_PI*3.2/17.3);
  /**
   * @brief Distance to goal following curve
   * 
   */
  double radialDistance_;
  /**
   * @brief Angle still to turn by vehicle during movement 
   * 
   */
  double delta_;
  /**
   * @brief radius of turning circle
   * 
   */
  double radius_;
  

  /**
   * @brief Target Speed of vehicle
   * 
   */
  const double Target_Speed = 0.2;

  /**
   * @brief Goal pipe for passing goals to
   * 
   */
  Pipes* goal_pipe;
  
/**
 * @brief Thread for Ackerman 
 * 
 */
std::thread Ackerman_Thread;  

};



#endif // ACKERMAN_H
