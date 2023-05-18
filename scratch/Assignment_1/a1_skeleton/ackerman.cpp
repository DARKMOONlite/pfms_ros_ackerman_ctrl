#include "ackerman.h"
#include <cmath>
#include "pipes.h"
#include <chrono>
#include <iostream>

using namespace std;

using pfms::commands::UGV;
using pfms::geometry_msgs::Goal;

Ackerman::Ackerman(){
    //Sets Platform Type and pipes
platform_ = pfms::PlatformType::ACKERMAN;
pipesPtr.reset(new Pipes(8,"/ugv_odo_buffer_seg","/ugv_odo_buffer_wsem","/ugv_odo_buffer_rsem",true));
Tolerance_ = 0.3;
}


bool Ackerman::reachGoal(){
    
    for(int i = 0; i < goal_.size(); i++){
    
    }
    double Estimate_Distance =  distanceToGoal();
    double Estimate_Time = timeToGoal();    
goal_pipe = new Pipes(5,"/ugv_buffer_seg","/ugv_buffer_wsem","/ugv_buffer_rsem");

while((x_distance_ > Tolerance_ || x_distance_ < -Tolerance_) || (y_distance_ > Tolerance_ || y_distance_ < -Tolerance_)){
    distanceToGoal();
 
    if(AngleToGoal(goal_.back().point,odometry_)==false){
    //if(AngleToGoal(goal_.at(current_goal).point,odometry_)==false){
        return(false);
    }
    else{
       for(int i = 0; i < 2; i++){
            UGV ugv {sequence_,0,-17.3*delta_,0.2};//"<repeats> <brake> <steering> <throttle> "
            
            sequence_++;
            goal_pipe->writeCommand(ugv);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if(!((x_distance_ > Tolerance_ || x_distance_ < -Tolerance_) || (y_distance_ > Tolerance_ || y_distance_ < -Tolerance_))){
       
                goto exit;
            }
        
        
        }

    }
}
    exit:; //When it reaches the goal
    current_goal++;
    Distance_Travelled_ += Estimate_Distance;
    Time_Travelled_ += Estimate_Time;

   for(int i = 0 ; i < 10; i ++){
    UGV ugv {sequence_,8000,0,0};
    sequence_++;
    goal_pipe->writeCommand(ugv);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
    return(true);


}

bool Ackerman::setGoal(pfms::geometry_msgs::Point goal){
    odometry_ = getOdometry();
    if(AngleToGoal(goal,odometry_)==false){
        return(false);
    }
    else{
    //Pipes pipes(5,"/goal_buffer_seg","/goal_buffer_wsem","/goal_buffer_rsem");
    Goal MockGoal{}; //Create a new goal
    MockGoal.point.x = goal.x; //Populate it with info
    MockGoal.point.y = goal.y;
    MockGoal.seq = num_goals;
    goal_.push_back(MockGoal);
   

    //Goal goal_pipe {static_cast<unsigned long>(num_goals),goal_.at(num_goals).point.x,goal_.at(num_goals).point.y};
    //pipes.writeCommand(goal_pipe);
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //num_goals++;
    return(true);  
    }
}

bool Ackerman::AngleToGoal(pfms::geometry_msgs::Point goal,pfms::nav_msgs::Odometry odometry){
    delta_ = 100; //random large number
    
    double theta =0;
    double beta = 0;
    double radius = 0;
    int TolleranceIncrement = 0;
    x_distance_ = goal.x - odometry.x;
    y_distance_ = goal.y - odometry.y;
    //double x_distance_array[] = {x_distance_,x_distance_+Tolerance_,x_distance_+Tolerance_,x_distance_-Tolerance_,x_distance_-Tolerance_};
    //double y_distance_array[] = {y_distance_,y_distance_+Tolerance_,y_distance_-Tolerance_,y_distance_+Tolerance_,y_distance_-Tolerance_};


    //for(int i = 0 ; i < 5; i ++){

        abs_distance_ = sqrt(pow(x_distance_, 2)+pow(y_distance_, 2));
    
     
        double alpha = atan(-y_distance_/x_distance_); //? This is the angle between the orientation of the car and the direction of the goal.
      


       theta = 2*M_PI+alpha+odometry.yaw;
   
        
        if( theta> M_PI){theta = theta - (2*M_PI);} // This is the angle from the direction the car is facing to the direction it needs to go
     
        delta_ = atan(2*Length_*sin(theta)/abs_distance_); 
        beta = (M_PI/2- theta);//(M_PI-2*alpha)/2; //180 degrees -2*alpha /2 sine  // This is just for finding the radius
 
    
        radius = abs(abs_distance_*sin(beta)/sin(2*theta)); //This is the Radius of the Circle that the car follows to the goal.
        
        if(radius<=1e-4){radius =abs(abs_distance_/2);} //fix for when triangle breaks
        radialDistance_ = abs(radius*2*theta); // This is the distance that the car will need to drive
    //     if(delta_ < Max_Steer_Angle || -delta_ < Max_Steer_Angle){ //If angle is within boundaries, break
    //         break;
    //     }
    // }



    if(delta_ > Max_Steer_Angle || -delta_ > Max_Steer_Angle){ //To big to get to thus return false

        return(false);
        
    }
    else{
    estimatedOdometry_ = {.seq=0, .x=goal.x,.y=goal.y,.yaw=alpha,.vx=0,.vy=0,};
        return(true);
        }
}


double Ackerman::distanceToGoal(void){
    //Returns the distance to the goal. this is the radial distance, not a stright line to the destination

    odometry_ = getOdometry();

    //AngleToGoal(goal_.at(current_goal).point,odometry_);
    AngleToGoal(goal_.back().point,odometry_);
    return(radialDistance_);
    //return(radialDistance_);

}


 double Ackerman::timeToGoal(){
     //Returns the time it will take to get to the goal_
     //Based of of the assumed speed of 2.91m/s




     return(radialDistance_/2.91);

 }

void Ackerman::Basic(unsigned int repeats, double brake, double steering, double throttle){
    // This is a copy of the a1 ackerman snippet. This is here for debugging.

    Pipes pipes(5,"/ugv_buffer_seg","/ugv_buffer_wsem","/ugv_buffer_rsem");

    unsigned long i = 0;
    /* produce messages */
    for(i = 0; i < repeats; i ++) {
        UGV ugv {i,brake,steering,throttle};//"<repeats> <brake> <steering> <throttle> "
        pipes.writeCommand(ugv);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cout << "wrote:" << i << endl;
    }
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                         pfms::nav_msgs::Odometry& estimatedGoalPose){
                                             // Checks the distance from the position origin to a goal
                                             //returns if it can reach the goal, the distance to goal, time to goal and the estimated final position of the vehicle
        if(AngleToGoal(goal,origin)==false){
            return false;
        }
        else{
            distance = radialDistance_;
            time = timeToGoal();
            estimatedGoalPose = estimatedOdometry_;
            return true;
        }



                                        }

