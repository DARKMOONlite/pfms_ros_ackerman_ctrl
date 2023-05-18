#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"

class Controller: public ControllerInterface
{
  //Default constructors should set all sensor attributes to a default value
  Controller();

  //See controllerinterface.h for more information
};

#endif // CONTROLLER_H
