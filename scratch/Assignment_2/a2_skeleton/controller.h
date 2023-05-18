#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <vector>
#include <utility>
#include <a1_types.h>
#include "pipes.h"
#include <ctime>
#include <queue>
#include <set>
#include <algorithm>
#include <utility>
#include <stack>
#include <limits>
#include <chrono>
#include <atomic>


class Controller: public ControllerInterface
{
  //Default constructors should set all sensor attributes to a default value

public:
  Controller();


  /**
   * @brief virtual function that creates threads for vehicles 
   * 
   */
  virtual void run()=0;
  /**
   * @brief this runs the vehicle
   * 
   */
  virtual void runThread()=0;

pfms::PlatformStatus status(void);


bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);




  /**
   * @brief Virtual Function, drives vehicle to goal
   * 
   * @return true 
   * @return false 
   */
  virtual bool reachGoal(int Goal_ID)=0;

  // /**
  //  * @brief Virtual Fucntion, sets a goal into the vehicle's goal vector
  //  * 
  //  * @param goal 
  //  * @return true 
  //  * @return false 
  //  */
  // virtual bool setGoal(pfms::geometry_msgs::Point goal) = 0;

  /**
   * @brief Returns the platform type
   * 
   * @return pfms::PlatformType 
   */
  pfms::PlatformType getPlatformType(void);

  /**
   * @brief Virtual Function, returns the distance from current position to the latest goal in the goal vector
   * 
   * @return double distance (m)
   */
  virtual double distanceToGoal(void)=0;

  /**
   * @brief Virtual Function, returns the time to the lastest goal
   * 
   * @return double time (s)
   */
  virtual double timeToGoal(void)=0;


  /**
   * @brief Sets the Tolerance for reachGoal() function
   * 
   * @param tolerance 
   * @return true 
   * @return false 
   */
  bool setTolerance(double tolerance);


  /**
   * @brief Returns the total time this vehicle has traveled
   * 
   * @return double 
   */
  double distanceTravelled(void);
  /**
   * @brief Returns the total distance this vehicle has traveled
   * 
   * @return double 
   */
  double timeTravelled(void);
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
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose)=0;





/**
 * @brief Function used by vehicles to update their distance and time values for other threads to access
 * 
 * @return true 
 * @return false 
 */
  virtual bool Update_DistTime()=0;
  





  


  /**
   * @brief uses a Pipe to get the Odometry from ros
   * 
   * @return pfms::nav_msgs::Odometry 
   */
 pfms::nav_msgs::Odometry getOdometry(void);






/**
 * @brief the current odometry values of the vehicle
 * 
 */
pfms::nav_msgs::Odometry odometry_;

protected:
 
/**
 * @brief The starting odometry Values
 */
pfms::nav_msgs::Odometry start_odometry_;
  /**
   * @brief Number of goals this vehicle has, increments when a goal is added
   */
  uint num_goals = 0; 
  /**
   * @brief Vector of goals for the vehicle to go to
   */
  std::vector<pfms::geometry_msgs::Goal> goals_;
  /**
   * @brief Total Distance Travelled
   * 
   */
  double Distance_Travelled_;

  /**
   * @brief Total Time Travelled for this vehicle
   * 
   */
  double Time_Travelled_;

  /**
   * @brief The time in (s) when the vehicle was created
   * 
   */
  std::chrono::steady_clock::time_point Start_Time_;
  /**
   * @brief Tolerance of goal position
   * 
   */
  double Tolerance_ = 0.2;
  /**
   * @brief a shared pointer used for reading the odometry
   * 
   */
  std::shared_ptr<Pipes> pipesPtr;
  /**
   * @brief Estimated Odometry, written to when determining which vehicle should go to which goal
   * 
   */
  pfms::nav_msgs::Odometry estimatedOdometry_;
 /**
  * @brief unsigned int so that all commands sent to the vehicle are read
  * 
  */
  unsigned int sequence_ = 0;
  /**
   * @brief distance between current position and goal position in the x axis
   * 
   */
  float x_distance_=0;
  /**
   * @brief distance between current position and goal position in the y axis
   * 
   */
  float y_distance_=0;
  /**
   * @brief Distance between current position and goal position sqrt(x_distance^2+y_distance^2)
   * 
   */
  float abs_distance_=0;
  /**
   * @brief The PlatformType of the vehicle
   * 
   */
  pfms::PlatformType platform_;
  /**
   * @brief Storage for when vehicle was created.
   * 
   */
  

  /**
   * @brief States what the current goal is, updating when a goal is reached.
   * 
   */
  unsigned int current_goal;

  //See controllerinterface.h for more information
/**
 * @brief Stores the current status of the vehicle, idle or active
 * 
 */
  pfms::PlatformStatus status_;
  /**
   * @brief This is the current goal the vehicle is working towards
   * 
   */
  pfms::geometry_msgs::Goal Cur_Goal;
  /**
   * @brief This is the mutex that gets atached to this object and controls data in and out
   * 
   */
  std::mutex thread_mtx_;
  /**
   * @brief This is a Conviditon variable that is triggered to make the main loop obtain data from threads without risk of race conditions
   * 
   */
  std::condition_variable thread_cv_;

/**
 * @brief Variable used to store the last Odometryt before updating Total Distance traveled and time
 * 
 */

  pfms::nav_msgs::Odometry Pre_Odometry;

/**
 * @brief Atomic to prevent unwated triggers of the condition variable
 * 
 */
  std::atomic<bool> ready_ = ATOMIC_VAR_INIT(true);





};


#endif // NAV_H
