#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pipes.h"
#include <deque>
using std::deque;
using std::vector;
using pfms::geometry_msgs::Goal;
using pfms::geometry_msgs::Point;
#include <thread>
#include <mutex>
#include <condition_variable> 


class Mission: public MissionInterface
{
public:
    /**
    The Default constructor
    @sa ControllerInterface and @sa MissionInterface for more information
    */
  Mission(std::vector<ControllerInterface*> controllers);
  /**
   * @brief sends code to the pipe to visually create the goals and also passes the goals to the controllers 
   * 
   * @param goals vector of points
   */
  void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform);
  // /**
  //  * @brief Runs the whole mission 
  //  * 
  //  * @return true if all goals are/can be reached
  //  * @return false if not
  //  */
  // bool runMission();

    /**
     * @brief Runs the mission, non blocking call
      * @return bool indicating mission complete (false if mission not possible OR aborted because it
      * can not be completed during execution)
     */
    bool run();

    /**
    @brief Retrurns mission status (indicating percentage of completion of entire task)
    @return percent of completed distance of entire mission
    */
    std::vector<unsigned int> status(void);




  /**
   * @brief Set the Mission Objective 
   * 
   * @param objective @enum mission::Objective::TIME or @enum mission::Objective::DISTANCE
   */
  void setMissionObjective(mission::Objective objective);
 /**
  * @brief Get a vector of distances traveled;
  * 
  * @return std::vector<double> 
  */
  std::vector<double> getDistanceTravelled();
 /**
  * @brief Get the Time a vehicle has been moving
  * 
  * @return std::vector<double> time (s)
  */
  std::vector<double> getTimeMoving();
  /**
   * @brief Associate each goal with a specific Vehicle, either the closest or fastest to reach
   * 
   * @return std::vector<unsigned int> vector of size(number of goals) with the index of the associated vehicle in each place
   */
  std::vector<unsigned int> getPlatformGoalAssociation();

  /**
   * @brief  Storage for the total distances each of the vehicle will need to travel
   * 
   */
  vector<double> Total_D;
  
 /**
  * @brief  Itterates through all goals to find shortest path
  * 
  * @param Controller_Index Index relating to the position of the vehicl in the controller vector
  * @param DorT Distance or Time calculation. 0 for distance, 1 for time
  * @return std::vector<Point>  returns the shortest path
  */
  std::vector<Point> FindBestPath(int Controller_Index,bool DorT);

  /**
   * @brief Old function not used. Calculates the distance througha deque of points 
   * 
   * @param path deque of points for vehicle to go between
   * @param Controller_Index Index of vehicle
   * @param cur_min_dist Used for comparison if at any point the distance becomes longer that cur_min_dist, returns negative
   * @return double Distance
   */
  double DistanceThroughPath(deque<int> path,int Controller_Index,double cur_min_dist);
  /**
   * @brief Checks that goals that cant be reached are skipped
   * 
   * @param path //vector of goals 
   * @param Controller_Index //controller index relating to controller vector
   * @return std::vector<Point> prunes goals that cannot be reached
   */
  std::vector<Point> BasicMethod(std::vector<Point> path, int Controller_Index);

/**
 * @brief Used for passing distance and odometry values to function
 * 
 */
struct path_pos {
  
  std::vector<double> distances;
  double total_dist = 0;
  std::vector<pfms::nav_msgs::Odometry> odos;
};
  /**
   * @brief This has replaced DistanceThroughPath in code only checks from known pos to known goal. much faster than old code
   * 
   * @param cur_pos current position of vehcile as well as current distance traveled and all previous distances
   * @param cur_goal Goal being aimed at
   * @param Controller_Index index of vehicle in question
   * @param DorT if you want to calculate distance or time, 0 is distance, 1 is time
   * @return path_pos 
   */
  path_pos Distance_through (path_pos cur_pos,pfms::geometry_msgs::Point cur_goal,int Controller_Index, bool DorT);

  /**
   * @brief Returns vector of all total distances vehicles have to travel
   * 
   * @return vector<double> 
   */
  vector<double> Max_Distance();


/**
 * @brief Storage for controllers
 * 
 */
std::vector<ControllerInterface*> controllers_; 

/**
 * @brief Distance or time calculation for finding the best path
 * 
 */
  bool DorT = 0;



private:
  /**
   * @brief Storage for ControllerInterfaces
   * 
   */
  
  /**
   * @brief private copy of goals for goals association
   * 
   */
  std::vector<pfms::geometry_msgs::Point> goals_; //!< A private copy of goals
  /**
   * @brief either mision::Objective::TIME or mision::Objective::DISTANCE. tells if system should check for distance or time.
   * 
   */
  mission::Objective objective_;
  /**
   * @brief pointer to a pipe that enables UAV & UGV commands to be sent to ros
   * 
   */
  Pipes* goal_pipes_;

  /**
   * @brief storage of the ordered goals for other functions
   * 
   */
  std::vector<std::vector<Point>> ordered_goals;



};

#endif // RANGERFUSION_H
