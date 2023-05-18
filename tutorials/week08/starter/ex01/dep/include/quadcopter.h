#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include "pipes.h"

namespace QuadcopterConst {
    const double TARGET_SPEED = 0.4;
}

//! UAV drone platform controller
class Quadcopter: public Controller
{
public:
  Quadcopter() :
    odo_pipe_(8,"/uav_odo_buffer_seg","/uav_odo_buffer_wsem","/uav_odo_buffer_rsem",true),
    cmd_pipe_(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem") {};

  bool reachGoal(void);
  /**
   * Calculates the angle needed for the quadcopter to reach a goal.
   * @return Always true - quadcopter has no unreachable goals.
   */
  bool calcNewGoal(void);
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time,
                                 pfms::nav_msgs::Odometry& estimatedGoalPose);

  pfms::PlatformType getPlatformType(void) { return pfms::QUADCOPTER; }
  pfms::nav_msgs::Odometry getOdometry(void) {
      update_odo(odo_pipe_);
      return platform_.odo;
  }
private:
  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  Pipes odo_pipe_;
  Pipes cmd_pipe_;
  long unsigned int cmd_pipe_seq_ = 0;

  //! Angle required for quadcopter to have a straight shot at the goal
  double target_angle_ = 0;
};

#endif // QUADCOPTER_H
