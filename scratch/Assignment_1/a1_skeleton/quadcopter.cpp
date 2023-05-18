#include "quadcopter.h"
#include <cmath>
#include <iostream>
#include "pipes.h"


using pfms::commands::UAV;
using pfms::geometry_msgs::Goal;
using std::cout;
using std::endl;
Quadcopter::Quadcopter(){
//Sets t
platform_ = pfms::PlatformType::QUADCOPTER;
//odometry_ = {.seq=0,.vx=0,.vy=0,.x=0,.y=0,.yaw=0}; Already Initialises odo in controller
pipesPtr.reset(new Pipes(8,"/uav_odo_buffer_seg","/uav_odo_buffer_wsem","/uav_odo_buffer_rsem",true));
 //move_pipes = new Pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
 //goal_pipe = new Pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
}


bool Quadcopter::reachGoal(){ //? Tries to get to the goal

    double Estimate_Distance =  distanceToGoal();
    double Estimate_Time = timeToGoal();

    //Pipes move_pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
    goal_pipe = new Pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
    UAV uav {0,0,0,0,0};
    while(((x_distance_ > Tolerance_ || x_distance_ < -Tolerance_) || (y_distance_ > Tolerance_ || y_distance_ < -Tolerance_)))
    { // While not within tollerances
        
     

        getOdometry();
        distanceToGoal();

        // float move_fb = 0.4*(x_distance_ * sin(odometry_.yaw) + y_distance_ * cos(odometry_.yaw))/abs_distance_;
        // float move_lr = 0.4*(y_distance_ * sin(odometry_.yaw) + x_distance_ * cos(odometry_.yaw))/abs_distance_;

        float move_lr = abs(0.4*(abs(x_distance_) * cos(abs(atan(y_distance_/x_distance_))-odometry_.yaw) + abs(y_distance_) * sin(abs(atan(y_distance_/x_distance_))-odometry_.yaw))/abs_distance_);
        float move_fb = abs(0.4*(abs(y_distance_) * cos(abs(atan(y_distance_/x_distance_))-odometry_.yaw) + abs(x_distance_) * sin(abs(atan(y_distance_/x_distance_))-odometry_.yaw))/abs_distance_);
        if(y_distance_<0){
            move_lr = -move_lr;
        }
        if(x_distance_<0){
            move_fb = -move_fb;
        }

        //./command_uav <repeats> <turn_l_r> <move_l_r> <move_u_d> <move_f_b>

       for(int i = 0; i < 5; i++){
        sequence_++;
        uav = {sequence_,-odometry_.yaw,move_lr,0,move_fb};
        //move_pipes.writeCommand(uav);
        goal_pipe->writeCommand(uav);
  

        for(int i = 0; i <goal_.size(); i++){
     
        }
 
      
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(!((x_distance_ > Tolerance_ || x_distance_ < -Tolerance_) || (y_distance_ > Tolerance_ || y_distance_ < -Tolerance_))){
   
            Distance_Travelled_ += Estimate_Distance;
            Time_Travelled_ += Estimate_Time;
            goto exit;
           
       }
       }


    }
    exit:;
    current_goal++;
    for(int i = 0; i <5;i++){
    uav = {sequence_,0,0,0,0};
    sequence_++;
    //move_pipes.writeCommand(uav);
    goal_pipe->writeCommand(uav);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return(true);
   
}




bool Quadcopter::setGoal(pfms::geometry_msgs::Point goal){

    // Pipes pipes(5,"/goal_buffer_seg","/goal_buffer_wsem","/goal_buffer_rsem");
    Goal MockGoal{}; //Create a new goal
    MockGoal.point.x = goal.x; //Populate it with info
    MockGoal.point.y = goal.y;
    MockGoal.seq = num_goals;
    goal_.push_back(MockGoal);

    // Goal goal_pipe {static_cast<unsigned long>(num_goals),goal_.at(num_goals).point.x,goal_.at(num_goals).point.y};
    // pipes.writeCommand(goal_pipe);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // num_goals++;

 

    return(1);



}

double Quadcopter::distanceToGoal(){ // Gets the Distance to the latest goal
    odometry_ = getOdometry();
    //x_distance_ = goal_.back().point.x - odometry_.x; 
    //y_distance_ = goal_.back().point.y - odometry_.y;
    x_distance_ = goal_.back().point.x - odometry_.x; 
    y_distance_ = goal_.back().point.y - odometry_.y;
    abs_distance_ = sqrt(pow(x_distance_, 2)+pow(y_distance_, 2));
    estimatedOdometry_ = {.seq=0,.x=goal_.back().point.x,.y=goal_.back().point.y,.yaw=0,.vx=0,.vy=0,};
    return(abs_distance_);
    
}


double Quadcopter::timeToGoal(){ // Returns the time that it'll take for the quadcopter to get to the goal

    double time_to_goal = distanceToGoal()/0.4;
    return(time_to_goal);
    

    }



void Quadcopter::Basic(unsigned int repeats, double turnl_r, double movel_r, double moveu_d, double movef_b ){
    // This is a copy of the a1_quadcopter snippet. This is here for debugging.
   Pipes pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");

    unsigned long i = 0;
    /* produce messages */
    for(i = 0; i < repeats; i ++) {
        UAV uav {i,turnl_r,movel_r,moveu_d,movef_b};
        pipes.writeCommand(uav);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        cout << "wrote msg:" << i << endl;
    }
    UAV uav {i,0,0,0,0};
    pipes.writeCommand(uav);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    cout << "wrote msg:" << i << endl;

}
  bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
    //Returns if the goal can be reached.
    // As well as the distance,time and final pose the vehicle will be in at the end.
    double x_distance = goal.x - origin.x; 
    double y_distance = goal.y - origin.y;
    distance= sqrt(pow(x_distance, 2)+pow(y_distance, 2));
    time = distance/0.4;
    estimatedOdometry_ = origin;
    estimatedGoalPose ={.seq=0,.x=goal.x, .y=goal.y,.yaw=0,.vx=0,.vy=0,};
    return(true);
    
    


                                        }