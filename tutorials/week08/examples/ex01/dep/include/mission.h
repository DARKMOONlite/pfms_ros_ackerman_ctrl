#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"

/**
 * \brief Orchestrates several platforms to complete a mission.
 * \sa MissionInterface
 * \bug Fails if any platform takes longer than 5 seconds to complete a chunk of missions (pipes close).
 */
class Mission: public MissionInterface
{
public:
    /**
    Default constructor - sets up empty internal state
     \param controllers List of controllers/platforms for this Mission to orchestrate.
    */
  Mission(std::vector<ControllerInterface*> controllers) : controllers_(std::move(controllers)) {
      time_moving_.resize(controllers_.size());
      distance_travelled_.resize(controllers_.size());
  }

  void setGoals(std::vector<pfms::geometry_msgs::Point*> goals) {
      goals_ = std::move(goals);
      goal_platform_assosciation_.resize(goals_.size());
      updateMission();
  }

  bool runMission();

  void setMissionObjective(mission::Objective objective) {
    objective_ = objective;
    updateMission();
  }

  std::vector<double> getDistanceTravelled() { return distance_travelled_; }

  std::vector<double> getTimeMoving() { return time_moving_; }

  std::vector<unsigned int> getPlatformGoalAssociation() { return goal_platform_assosciation_; }
private:
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point*> goals_; //!< A private copy of goals
  mission::Objective objective_ = mission::Objective::DISTANCE;
  std::vector<double> time_moving_;
  std::vector<double> distance_travelled_;
  std::vector<unsigned int> goal_platform_assosciation_; //! @sa MissionInterface::getPlatformGoalAssociation

  /**
   * Task - send a platform to a goal.
   */
  struct GoalTarget {
      //! The goal to move towards.
      unsigned int goal;
      //! The platform which should do the moving.
      unsigned int platform;
  };
  std::vector<GoalTarget> goal_order_;

  /**
   * Calculates the goal ordering and platform association, given the current goals and objectives.
   */
  void updateMission();
};

#endif // MISSION_H
