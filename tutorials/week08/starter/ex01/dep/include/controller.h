#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pipes.h>

//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    pfms::geometry_msgs::Point location;
    //! Radius of tolerance
    double tolerance = 0; //might not actually be set by the caller
    //! distance to goal
    double distance = 0;
    //! time to goal
    double time = 0;
};
//! Information about the current state of the platform
struct PlatformStats {
    //! pose
    pfms::nav_msgs::Odometry odo;
    //! Total distance travelled for this program run
    double distance_travelled;
    //! Total time spent travelling for this program run
    double time_travelled; //NOTE: only the Audi can reach 88mph, should upgrade drone for nonzero /j
};

/**
 * \brief Shared functionality/base class for platform controllers
 *
 * Platforms need to implement:
 * - Controller::calcNewGoal (and updating GoalStats)
 * - ControllerInterface::reachGoal (and updating PlatformStats)
 * - ControllerInterface::checkOriginToDestination
 * - ControllerInterface::getPlatformType
 * - ControllerInterface::getOdometry (and updating PlatformStats.odo)
 */
class Controller: public ControllerInterface
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller() {
      platform_ = (PlatformStats) {
          .odo = { 0 },
          .distance_travelled = 0,
          .time_travelled = 0,
      };
  };

  /**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  virtual bool calcNewGoal() = 0;

  //ControllerInterface functions
  bool setGoal(pfms::geometry_msgs::Point goal) {
      goal_.location = goal;
      return calcNewGoal();
  }
  bool setTolerance(double tolerance) {
      goal_.tolerance = tolerance;
      return calcNewGoal();
  }
  double distanceToGoal(void) { return goal_.distance; }
  double timeToGoal(void) { return goal_.time; }
  double distanceTravelled(void) { return platform_.distance_travelled; }
  double timeInMotion(void) { return platform_.time_travelled; }
protected:
  /**
   * Checks if the goal has been reached.
   *
   * Update own odometry before calling!
   * @return true if the goal is reached
   */
  bool goalReached() {
      double dx = goal_.location.x - platform_.odo.x;
      double dy = goal_.location.y - platform_.odo.y;

      return (std::abs(dx) < goal_.tolerance) && (std::abs(dy) < goal_.tolerance);
  }
  /**
   * Updates the internal odometry
   *
   * Sometimes the pipes can give all zeros on opening, this has a little extra logic to ensure only valid data is
   * accepted
   * @param odo_pipe The platform's odometry pipe
   */
  void update_odo(Pipes& odo_pipe) {
      pfms::nav_msgs::Odometry odo = { .seq = 0 };
      while (odo.seq == 0)
          odo = odo_pipe.getOdo();
      platform_.odo = odo;
  }
  //stats
  GoalStats goal_;
  PlatformStats platform_;
};

#endif // CONTROLLER_H
