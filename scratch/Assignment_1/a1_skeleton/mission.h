#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pipes.h"

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
  void setGoals(std::vector<pfms::geometry_msgs::Point*> goals);
  /**
   * @brief Runs the whole mission 
   * 
   * @return true if all goals are/can be reached
   * @return false if not
   */
  bool runMission();
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
private:
  /**
   * @brief Storage for ControllerInterfaces
   * 
   */
  std::vector<ControllerInterface*> controllers_; 
  /**
   * @brief private copy of goals for goals association
   * 
   */
  std::vector<pfms::geometry_msgs::Point*> goals_; //!< A private copy of goals
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
};

#endif // RANGERFUSION_H
