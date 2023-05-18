#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <vector>
#include <utility>
#include <a1_types.h>

#include <ctime>
#include <queue>
#include <set>
#include <algorithm>
#include <utility>
#include <stack>
#include <limits>
#include <chrono>
#include <atomic>

/*!
 *  \brief     General Control Class, contains many usefull functions
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived autonomous vehicle controller classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Sebastian Schroder
 *  \version   1.01-2
 *  \date      2022-04-15
 *  \pre       none
 *  \bug       none reported as of 2022-04-15
 *  \warning   students MUST NOT change this class (the header file)
 */
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
 * @brief Callback for Odometry from Ros Topic
 * 
 * @param msg contains position, orientation & velocity data; rosmsg nav_msgs/Odometry
 */
 void getOdometry(const nav_msgs::OdometryConstPtr &msg);
/**
 * @brief Callback for LaserScan Data
 * 
 * @param msg Contains laserscan data; rosmsg sensor_msgs/LaserScan
 */
void getLaserScan(const sensor_msgs::LaserScanConstPtr& msg);




/**
 * @brief Callback for Sonar Msg
 * 
 * @param msg Contains Sonar Data; rosmsg sensor_msgs/Range
 */
void getSonar(const sensor_msgs::RangeConstPtr &msg);

/**
 * @brief Creates and visulises shapes in the gazebo simulation
 * 
 * @param points point to create shape at
 * @param shape what type of shape (0-10)
 */
void Visualise(std::vector<geometry_msgs::Point> points, int shape);

/**
 * @brief Service Server for starting robot
 * 
 * @param req request to start robot
 * @param resp response, responds if a code is visable from starting position, as well as a custom message
 * @return true 
 * @return false 
 */
bool StartVehicle(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

/**
 * @brief Sends command to simulation using various ros topics
 * 
 * @param brake Brake Value 10,000 - 0
 * @param steering steering value.
 * @param throttle Recommended value 0.2;
 */
virtual void sendCMD(double brake, double steering, double throttle) = 0;
/**
 * @brief check if current distance from goal is within square tollerances
 * 
 * @return true 
 * @return false 
 */
bool withinTolerance();

/**
 * @brief Checks if the provided goal is within a range of the 2 closest cones
 * 
 * @param Goal Passed goal location
 * @return true 
 * @return false 
 */
bool GoalWithinCones(pfms::geometry_msgs::Goal Goal);

/**
 * @brief Function to determine if goal is within 2 cones & allows unit tests to be run on it.
 * 
 * @param Goal Location of goal
 * @param Cones Pair of 2 Cones that represent the boundaries that the goal should be within
 * @param location location of Vehicle, needed for testing
 * @return true 
 * @return false 
 */
bool WithinTriangle(pfms::geometry_msgs::Goal Goal,std::vector<geometry_msgs::Point> Cones,pfms::nav_msgs::Odometry location);
/**
 * @brief Converts new Pose data to old pfms:: goal data structure
 * 
 * @param goal 
 * @return pfms::geometry_msgs::Goal 
 */
pfms::geometry_msgs::Goal ConvertGoal2pfms(geometry_msgs::Pose goal);
/**
 * @brief Converts old goal data structure to new Pose data
 * 
 * @param Goal 
 * @return geometry_msgs::Pose 
 */
geometry_msgs::Pose ConvertGoalFpfms(pfms::geometry_msgs::Goal Goal);

/**
 * @brief Converts new Odometry style to old one.
 * 
 * @return pfms::nav_msgs::Odometry 
 */
pfms::nav_msgs::Odometry ConvertOdo2pfms(nav_msgs::Odometry odo);

/**
 * @brief Used in Unittests to force the Running check on or off
 * 
 * @param value what value to set the running_ atomic variable to
 */
void OverrideRunningCheck(bool value);
/**
 * @brief Getter for if vehicle is running or not. Used in UTests
 * 
 * @return true 
 * @return false 
 */
bool ReadRunningCheck();

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
 * @brief Storage for all known locations of cones. new cones are compared against these to check if they clash
 * 
 */
  std::vector<geometry_msgs::Point> knownCones;

/**
 * @brief Variable used to store the last Odometryt before updating Total Distance traveled and time
 * 
 */

  pfms::nav_msgs::Odometry Pre_Odometry;




 
/**
 * @brief Publishers for UGV Communication
 * 
 */
  ros::Publisher brake_cmd_pub_, steering_cmd_pub_, throttle_cmd_pub_; //! Brake, Steering and Throttle (UGV) publishers
  /**
   * @brief Publisher for Visualising messages to Gazebo simulation
   * 
   */
ros::Publisher viz_pub_;//! Visualisation Marker publisher
/**
 * @brief Subscribers for radar, laser and odometry Values
 * 
 */
   ros::Subscriber sub1_,sub2_,sub3_;  // Few subscribers
   /**
    * @brief Service used to start vehicle motion
    * 
    */
  ros::ServiceServer start_vehicle;
  /**
   * @brief Atomic bool to set vehicle running or not
   * 
   */
  std::atomic<bool> Running_ = ATOMIC_VAR_INIT(false);


  /**
   * @brief Storage for pointer to laser data, due to it's large size it is unrealistic to copy it.
   * 
   */
LaserProcessing* laserProcessingPtr_;
/**
 * @brief Increments whenever a new object is added to the gazebo simulation so they all have unique ids
 * 
 */
  unsigned int marker_counter = 0;

/**
 * @brief Tollerance for Cone visualisations to avoid cone drift
 * 
 */
  double cone_toll_ = 3;

/**
 * @brief Tollerance 
 * 
 */
  double sonar_toll_= 2.5;

/**
 * @brief Atomic to prevent unwated triggers of the condition variable
 * 
 */
  std::atomic<bool> ready_ = ATOMIC_VAR_INIT(true);
/**
 * @brief  Updated every laser scan and used to confirm if sonar is hitting object or cone
 * 
 */
  geometry_msgs::Point nearest_cone_;



};


#endif // NAV_H
