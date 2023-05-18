#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

class Quadcopter: public Controller
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter();

  bool reachGoal();


};

#endif // QUADCOPTER_H
