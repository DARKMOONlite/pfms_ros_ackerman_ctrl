#include "controller.h"
#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>


using pfms::geometry_msgs::Goal;



Controller::Controller(){
    //Just initialize the odometry to 0;
odometry_ = {.seq=0,.x=0,.y=0,.yaw=0,.vx=0,.vy=0,}; // Get base odometry Values for later comparison
start_odometry_ = {.seq=0,.x=0,.y=0,.yaw=0,.vx=0,.vy=0,}; // These have to be like this for the Unit tests. if you run getOdometry it doesn't work.
start_time_ = std::time(0); // Get time for later functions
current_goal=0;
}


pfms::PlatformType Controller::getPlatformType(void){// Just return the platform type
    
    return(platform_);
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){


    pfms::nav_msgs::Odometry odo;

    for(int i = 0; i <1; i++) { //! probably will remove this for loop
        odo =  pipesPtr->getOdo();
        odometry_ = odo;
        estimatedOdometry_ = odo;
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }
    return(odometry_);
}

bool Controller::setTolerance(double tolerance){// Sets the tolerance of the vehicle
Tolerance_ = tolerance;


return(1);

}



 double Controller::distanceTravelled(void){//Returns the distance traveled in comparison to the start_odometry_
    
     return(Distance_Travelled_);
 }
double Controller::timeInMotion(void){//Returns the Total time spent moving 
    return(Time_Travelled_);
    
   

}



