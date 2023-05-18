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
Quadcopter::~Quadcopter(){
    delete goal_pipe;
    pipesPtr.reset();
    
    Quadcopter_Thread.join();
}

void Quadcopter::run(){
   
    status_= pfms::RUNNING; 
     Quadcopter_Thread = std::thread(&Quadcopter::runThread,this);
    
}

void Quadcopter::runThread(){

    
    std::cout << "Running thread" << std::endl;
    status_= pfms::RUNNING;  

  

    bool reached =0;
    for(int i = 0; i < goals_.size(); i++){

            reached = reachGoal(i); //Runs the vehicle towards the goal

        if(reached==0){ // if for any reason the goal isn't reached then break
            std::cout << "Goal couldn't be reached" << std::endl;
            break;
            }

        
    }
    
    status_ = pfms::IDLE;
    
    std::cout << "End of Thread" << std::endl;
    return;
}


bool Quadcopter::reachGoal(int Goal_ID){ //? Tries to get to the goal
    std::unique_lock<std::mutex> lck(thread_mtx_);

    //std::cout << "Trying to reach goal" << std::endl;
    Cur_Goal = goals_.at(Goal_ID);
     double Estimate_Distance =  distanceToGoal();
    // double Estimate_Time = timeToGoal();
     Pre_Odometry = getOdometry();
    //Pipes move_pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
    goal_pipe = new Pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");
    UAV uav {0,0,0,0,0};

    while( std::abs(x_distance_) > Tolerance_|| std::abs(y_distance_) > Tolerance_)
    { // While not within tollerances

        
  
       getOdometry();
       double cur_dist = distanceToGoal();
        

        // Used to determine the speed the vehicle should travel
        if(cur_dist > LowPrecisionRange){Target_Speed= MX_Speed;}
        else{if(cur_dist> HighPrecisionRange){Target_Speed = LP_Speed;}
        else{Target_Speed = HP_Speed;}
        }
        
        
        double angle = atan2(y_distance_,x_distance_);

        double move_fb = Target_Speed * cos(angle - odometry_.yaw);
        double move_lr = Target_Speed * sin(angle - odometry_.yaw);
        //./command_uav <repeats> <turn_l_r> <move_l_r> <move_u_d> <move_f_b>
  
       for(int i = 0; i < 2; i++){
            sequence_++;

            uav = {sequence_,-odometry_.yaw,move_lr,0,move_fb};
            goal_pipe->writeCommand(uav);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));


        if(!(std::abs(x_distance_) > Tolerance_|| std::abs(y_distance_) > Tolerance_)){
            goto exit;
       }
       }
       
        Update_DistTime();
        ready_ = true;
        lck.unlock( );
        thread_cv_.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        lck.lock();
        
    }


    exit:;
    Update_DistTime();
  
    

    current_goal++;
    for(int i = 0; i <5;i++){
        uav = {sequence_,0,0,0,0};
        sequence_++;
        goal_pipe->writeCommand(uav);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
        ready_ = true;
        lck.unlock( );
        thread_cv_.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return(true);
   
}




bool Quadcopter::setGoal(pfms::geometry_msgs::Point goal){
    Goal MockGoal{}; //Create a new goal
    MockGoal.point.x = goal.x; //Populate it with info
    MockGoal.point.y = goal.y;
    //MockGoal.point = goal;
    MockGoal.seq = num_goals;
    goals_.push_back(MockGoal);
    return(1);
}

double Quadcopter::distanceToGoal(){ // Gets the Distance to the latest goal

    odometry_ = getOdometry();
    //x_distance_ = goal_.back().point.x - odometry_.x; 
    //y_distance_ = goal_.back().point.y - odometry_.y;
    x_distance_ = Cur_Goal.point.x - odometry_.x; 
    y_distance_ = Cur_Goal.point.y - odometry_.y;
    abs_distance_ = sqrt(pow(x_distance_, 2)+pow(y_distance_, 2));
    estimatedOdometry_ = {.seq=0,.x=Cur_Goal.point.x,.y=Cur_Goal.point.y,.yaw=0,.vx=0,.vy=0,};
    return(abs_distance_);
    
}


double Quadcopter::timeToGoal(){ // Returns the time that it'll take for the quadcopter to get to the goal

    double time_to_goal = distanceToGoal()/Target_Speed;
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
    estimatedGoalPose ={.seq=0,.x=goal.x, .y=goal.y,.yaw=origin.yaw,.vx=0,.vy=0,};
    return(true);
    
    


                                        }



bool Quadcopter::Update_DistTime(){
    getOdometry();
    Distance_Travelled_ += sqrt(pow((odometry_.x-Pre_Odometry.x),2)+pow((odometry_.y-Pre_Odometry.y),2));
    Pre_Odometry = odometry_;
    std::chrono::duration<double> diff = std::chrono::steady_clock::now() - Start_Time_;
         Time_Travelled_ = diff.count();
        //  std::cout << "Distance Traveled" << Distance_Travelled_ << std::endl;
        //  std::cout << "Time_Traveled" << Time_Travelled_ << std::endl;
            
         return(1);
}
