#include "controller.h"
#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>
#include <utility>


using pfms::geometry_msgs::Goal;
using std::vector;
using std::deque;


Controller::Controller(){
    //Just initialize the odometry to 0;
odometry_ = {.seq=0,.x=0,.y=0,.yaw=0,.vx=0,.vy=0,}; // Get base odometry Values for later comparison
start_odometry_ = {.seq=0,.x=0,.y=0,.yaw=0,.vx=0,.vy=0,}; // These have to be like this for the Unit tests. if you run getOdometry it doesn't work.
Start_Time_ = std::chrono::steady_clock::now();
current_goal=0;

status_ = pfms::IDLE;
Time_Travelled_ = 0;
Distance_Travelled_ = 0;
}






bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals){

Goal MockGoal;
for(int i = 0; i<goals.size(); i++){
    
    MockGoal.point = goals.at(i);
    MockGoal.seq = i;
    goals_.push_back(MockGoal);
    

}
Cur_Goal = goals_.at(0);
return(true);
}

double Controller::distanceTravelled(void){
    //std::cout << "Waiting at cv" << std::endl;
    std::unique_lock<std::mutex> lck(thread_mtx_);
    // thread_cv_.wait(lck, [&](){return ready_==true;});
    
    thread_cv_.wait_for(lck,std::chrono::seconds(1),[&](){return ready_==true;});


    //std::cout << "Distance Traveled: " <<Distance_Travelled_ << std::endl;
    ready_ = false;
    lck.unlock();
    return(Distance_Travelled_);}

double Controller::timeTravelled(void){return(Time_Travelled_);}
// Just return the platform type
pfms::PlatformType Controller::getPlatformType(void){return(platform_);}

pfms::PlatformStatus Controller::status(void){
    std::unique_lock<std::mutex> lck(thread_mtx_);
    //std::cout << "controller Status function called " << status_ << std::endl;
    return(status_);
    }

pfms::nav_msgs::Odometry Controller::getOdometry(void){


    pfms::nav_msgs::Odometry odo;

    
        odo =  pipesPtr->getOdo();
        odometry_ = odo;
        estimatedOdometry_ = odo;
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    
    return(odometry_);
}
// Sets the tolerance of the vehicle
bool Controller::setTolerance(double tolerance){Tolerance_ = tolerance;
return(1);
}


//Returns the distance traveled in comparison to the start_odometry_






