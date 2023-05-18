#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include <cmath>

namespace AckermanConst {
    // apparently in C++ you can just.. do this, in a header? breaks my C brain
    const double TARGET_SPEED = 2.91;
    // While the Audi gets up to speed much faster at 0.2, this was specified in assignment FAQs and Teams messages
    // as the required value. If you want to see the time metrics be accurate, 0.2 is better.
    const double TARGET_THROTTLE = 0.1;
    const double MAX_BRAKE_FORCE = 8000.0;
    const double STEERING_RATIO = 17.3;
    const double LOCK_LOCK_REVS = 3.2;
    const double MAX_STEER_ANGLE = M_PI * LOCK_LOCK_REVS / STEERING_RATIO;
    const double WHEELBASE = 2.65;
}

//! UGV audi platform controller
class Ackerman: public Controller
{
public:
  Ackerman() :
    odo_pipe_(8,"/ugv_odo_buffer_seg","/ugv_odo_buffer_wsem","/ugv_odo_buffer_rsem",true),
    cmd_pipe_(5,"/ugv_buffer_seg","/ugv_buffer_wsem","/ugv_buffer_rsem") {};

  bool reachGoal(void);
  /**
   * Calculates the required steering angle for the audi to successfully reach a goal.
   * @return true if the required steering is within the audi's turning circle, false otherwise.
   */
  bool calcNewGoal(void);
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                double& distance, double& time,
                                pfms::nav_msgs::Odometry& estimatedGoalPose);

  pfms::PlatformType getPlatformType(void) { return pfms::ACKERMAN; }
  pfms::nav_msgs::Odometry getOdometry(void) {
      update_odo(odo_pipe_);
      return platform_.odo;
  }
private:
  void sendCmd(double brake, double steering, double throttle);

  Pipes odo_pipe_;
  Pipes cmd_pipe_;
  long unsigned int cmd_pipe_seq_ = 0;

  //! *steering angle* for the audi to make a turn that intersects with the goal.
  double target_angle_ = 0;
};

#endif // ACKERMAN_H
