#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"


class Controller: public ControllerInterface
{
  //Default constructors should set all sensor attributes to a default value
  Nav();
public:
  Controller();



  virtual bool reachGoal(void)=0;

  bool setGoal(void);

  pfms::PlatformType getPlatformType(void);

  double distanceToGoal(void);

  double timeToGoal(void);

  bool setTolerance(double tolerance);

  double distanceTravelled(void);

  //double timeTravelled(void);

  pfms::nav_msgs::Odometry getOdometry(void)


  //See controllerinterface.h for more information
};


#endif // NAV_H
