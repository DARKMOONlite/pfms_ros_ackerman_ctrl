#include "controller.h"
#include <cmath>


/**
 * \brief Shared functionality/base class for platform controllers
 *
 */
Controller::Controller() :
    distance_travelled_(0),
    time_travelled_(0),
    cmd_pipe_seq_(0)
{
    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
};


bool Controller::setGoal(geometry_msgs::Point goal) {
  goal_.location = goal;
  return calcNewGoal();
}

bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;
}
double Controller::timeToGoal(void) {
    return goal_.time;
}
double Controller::distanceTravelled(void) {
    return distance_travelled_;
}
double Controller::timeInMotion(void) {
    return time_travelled_;
}

bool Controller::goalReached() {
    double dx = goal_.location.x - odo_.pose.pose.position.x;
    double dy = goal_.location.y - odo_.pose.pose.position.y;

    return (pow(pow(dx,2)+pow(dy,2),0.5) < tolerance_);
}


nav_msgs::Odometry Controller::getOdometry(void){

//    odo_.seq = 0; // We set seq to zero in initialisation

//    while (odo_.seq == 0){ // If the seq number is zero ignore it and request another one
//      odo_ = odo_pipesPtr_->getOdo();
//    }
//    return odo_;
  std::unique_lock<std::mutex> lck (odoMtx_);
  return odo_;

}

pfms::PlatformType Controller::getPlatformType(void){
    return type_;
}



