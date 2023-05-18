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
Ackerman::~Ackerman(){
    delete goal_pipe;
    pipesPtr.reset();
}

void Ackerman::run(){
    status_= pfms::RUNNING; 

    Ackerman_Thread = std::thread(&Ackerman::runThread,this);


    return;

}


void Ackerman::runThread(){
    std::cout << "Running thread" << std::endl;
 status_= pfms::RUNNING;

    bool reached =0;
    std::cout << "num of goals" << goals_.size() << std::endl;
    for(int i = 0; i < goals_.size(); i++){
            reached = reachGoal(i);

        if(reached==0){
            std::cout << "Goal couldn't be reached" << std::endl;
            double testdistance = 0.0;
            double testtime = 0.0;
            pfms::nav_msgs::Odometry testodo;
            bool check = checkOriginToDestination(getOdometry(),goals_.at(i).point,testdistance,testtime,testodo);
            std::cout << "check: " << check << std::endl;
            std::cout << "Distance: " << testdistance << std::endl;
            break;
            }

        
    }


    status_ = pfms::IDLE;

    // std::cout << "End of Thread" << std::endl;
    return;

}



bool Ackerman::reachGoal(int Goal_ID){
    std::unique_lock<std::mutex> lck(thread_mtx_);
    //! Set desired goal to Goal relating to Goal_ID

    Cur_Goal = goals_.at(Goal_ID);
    std::cout << " goal:" << goals_.at(Goal_ID).point.x << " " << goals_.at(Goal_ID).point.y<< std::endl;
 
    double T_Distance =  distanceToGoal();
    double T_Time = timeToGoal();    
    double Break = 0;
    double Break_Time[] = {T_Distance/3, T_Distance/5.5};



    goal_pipe = new Pipes(5,"/ugv_buffer_seg","/ugv_buffer_wsem","/ugv_buffer_rsem");

    while((std::abs(x_distance_) > Tolerance_|| std::abs(y_distance_) > Tolerance_)){
        getOdometry();
 
        if(AngleToGoal(Cur_Goal.point,odometry_)==false){
    //if(AngleToGoal(goal_.at(current_goal).point,odometry_)==false){
            std::cout << "Returning False" << std::endl;
                if(!(std::abs(x_distance_) > Tolerance_|| std::abs(y_distance_) > Tolerance_)){
       
                    break;
                }
            return(false);
        }
        else{
                // double cur_speed = sqrt(pow(odometry_.vx,2)+pow(odometry_.vy,2));
                // std::cout << "              cur_speed: " << cur_speed << std::endl;       
                // double speed = 
                //std::cout << "          radialDistance_: " << radialDistance_ << std::endl;
                if(radialDistance_ < Break_Time[0]){Break=800;}
                else{if(radialDistance_ < Break_Time[0]){Break=2000;}
                else{Break = 0;}}
                UGV ugv {sequence_,Break,17.3*delta_,0.2};
                getOdometry();
                // UGV ugv {sequence_,0,17.3*delta_,speed};//"<repeats> <brake> <steering> <throttle> "
            
                sequence_++;
                goal_pipe->writeCommand(ugv);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                if(!(std::abs(x_distance_) > Tolerance_|| std::abs(y_distance_) > Tolerance_)){
       
                    break;
                }
        
        Update_DistTime();
        ready_ = true;
        lck.unlock( );
        thread_cv_.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        lck.lock();

    }
}
    // exit:; //When it reaches the goal


   for(int i = 0 ; i < 10; i ++){
    UGV ugv {sequence_,10000,0,0};
    sequence_++;
    goal_pipe->writeCommand(ugv);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
    std::cout <<"Goal Reached" << std::endl;
    current_goal++;
    Update_DistTime();
    ready_ = true;
        lck.unlock( );
        thread_cv_.notify_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
    goals_.push_back(MockGoal);
   

    //Goal goal_pipe {static_cast<unsigned long>(num_goals),goal_.at(num_goals).point.x,goal_.at(num_goals).point.y};
    //pipes.writeCommand(goal_pipe);
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //num_goals++;
    return(true);  
    }
}

bool Ackerman::AngleToGoal(pfms::geometry_msgs::Point goal,pfms::nav_msgs::Odometry odometry){
    
    
    double theta =0,beta =0;
    
    x_distance_ = goal.x - odometry.x;
    y_distance_ = goal.y - odometry.y;

        abs_distance_ = sqrt(pow(x_distance_, 2)+pow(y_distance_, 2));
        double alpha = atan2(y_distance_,x_distance_); //? This is the angle between the orientation of the car and the direction of the goal.
    //    std::cout << "alpha: " << alpha << std::endl;

    theta = alpha-odometry.yaw;
    if(theta >M_PI){theta-=2*M_PI;} 
    if(theta < -M_PI){theta +=2*M_PI;}

    //    theta = 2*M_PI+alpha+odometry.yaw;
    //     if( theta> M_PI){theta = theta - (2*M_PI);} // This is the angle from the direction the car is facing to the direction it needs to go
        // std::cout << "  Theta: " << theta << std::endl;
        
        delta_ = atan(2*Length_*sin(theta)/abs_distance_); //? Equation from Ackerman Steering Model
        // std::cout << "      Delta: " << delta_ << std::endl;

        //? Everything below here is used to calculate the distance to travel
        beta = (M_PI/2- theta);//(M_PI-2*alpha)/2; //180 degrees -2*alpha /2 sine  // This is just for finding the radius
        radius_ = abs(abs_distance_*sin(beta)/sin(2*theta)); //This is the radius_ of the Circle that the car follows to the goal.
        
        if(radius_<=1e-4){radius_ =abs(abs_distance_/2);} //fix for when triangle breaks
       
        
        if(isinf(radius_)){ //? if the radius_ is infinite, drive straight 
            radialDistance_ = abs_distance_;
        }
        else{
            radialDistance_ = abs(radius_*2*theta); // This is the distance that the car will need to drive      
        }
        
    
    //     if(delta_ < Max_Steer_Angle || -delta_ < Max_Steer_Angle){ //If angle is within boundaries, break
    //         break;
    //     }
    // }

    double new_yaw = (2*alpha-odometry.yaw); 
    if(new_yaw>M_PI){new_yaw-=2*M_PI;} //Standardises yaw so it is between 180 & -180 degrees
    if(new_yaw<-M_PI){new_yaw+=2*M_PI;}
    

    if(delta_ > Max_Steer_Angle || -delta_ > Max_Steer_Angle){ //To big to get to thus return false

        return(false);
        
    }
    else{
           
    estimatedOdometry_ = {.seq=0, .x=goal.x,.y=goal.y,.yaw=new_yaw,.vx=0,.vy=0,};
        return(true);
        }
}


double Ackerman::distanceToGoal(void){
    //Returns the distance to the goal. this is the radial distance, not a stright line to the destination

    std::cout << "Current Goal Pos: " << Cur_Goal.point.x << " " << Cur_Goal.point.y << std::endl;
    odometry_ = getOdometry();
    std:: cout << "Odometry is: " << odometry_.x<< " " <<odometry_.y << " "<< odometry_.yaw << std::endl;
    //AngleToGoal(goal_.at(current_goal).point,odometry_);
    AngleToGoal(Cur_Goal.point,odometry_);
    std::cout << " Distance to goal is" << radialDistance_ << std::endl;
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
            distance = -1.0;
            time = -1.0;
            return false;
        }
        else{
            distance = radialDistance_;
            time = timeToGoal();
            estimatedGoalPose = estimatedOdometry_;
            return true;
        }



                                        }




    bool Ackerman::Update_DistTime(){
    getOdometry();
    Distance_Travelled_ += sqrt(pow((odometry_.x-Pre_Odometry.x),2)+pow((odometry_.y-Pre_Odometry.y),2));
    Pre_Odometry = odometry_;
    std::chrono::duration<double> diff = std::chrono::steady_clock::now() - Start_Time_;
         Time_Travelled_ = diff.count();
        //  std::cout << "Distance Traveled" << Distance_Travelled_ << std::endl;
        //  std::cout << "Time_Traveled" << Time_Travelled_ << std::endl;

         return(1);



    }

