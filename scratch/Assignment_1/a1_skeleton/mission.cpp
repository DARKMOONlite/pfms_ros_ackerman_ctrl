
#include <vector>
#include "mission.h"
#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using pfms::geometry_msgs::Goal;

Mission::Mission(std::vector<ControllerInterface*> controllers){
controllers_ = controllers;





}



void Mission::setGoals(std::vector<pfms::geometry_msgs::Point*> goals){
    //sets goals to all controllers
    
    goals_ = goals;

    goal_pipes_ = new Pipes(5,"/goal_buffer_seg","/goal_buffer_wsem","/goal_buffer_rsem");
    //Pipes goal_pipes_(5,"/goal_buffer_seg","/goal_buffer_wsem","/goal_buffer_rsem");


    for(int i = goals.size()-2; i < goals.size(); i++){ // for each goal in the vector

    // send the goals to the pipe to visualise them

    Goal goal {static_cast<unsigned long>(i),
        {goals.at(i)->x,goals.at(i)->y} //<seq_num> <goal_1_x> <goal_1_y>
         };


    //goal_pipes_.writeCommand(goal);
    goal_pipes_->writeCommand(goal);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
 
    }

    return;

}

  bool Mission::runMission(){ //Runs the mission

      std::vector<unsigned int> GoalAssoc = getPlatformGoalAssociation(); //Take each goal and give them to either the closest vehicle by time or distance
      bool Possible;
      //GoalAssoc is a vector containing which vehicle should go to each goal
      if(GoalAssoc.size() != goals_.size()){return false; //Catches cases where no vehicle can reach a goal
      }
      for(unsigned int i = 0; i < GoalAssoc.size();i++){

          Possible = controllers_.at(GoalAssoc.at(i))->setGoal(*goals_.at(i)); // set the goals
            if(Possible==0){
                return(false);
            }
          controllers_.at(GoalAssoc.at(i))->reachGoal();
          
      }
      return(true);

  }

  void Mission::setMissionObjective(mission::Objective objective){
      objective_ = objective;
      //sets either time or Distance
      
  }

  std::vector<double> Mission::getDistanceTravelled(){ // returns vector containing the distances each vehicle has traveled
      std::vector<double> Distances;
      for(int i = 0; i < controllers_.size(); i++){
          auto Distance = controllers_.at(i)->distanceTravelled();
          Distances.push_back(Distance);
      }
      return(Distances);
  }

  std::vector<double> Mission::getTimeMoving(){// returns vector containing the time over which each vehicle has traveled
      std::vector<double> Times;
      for(int i = 0; i < controllers_.size();i++){
          auto Time = controllers_.at(i)->timeInMotion();
          Times.push_back(Time);
      }
      return(Times);
  }

  std::vector<unsigned int> Mission::getPlatformGoalAssociation(){ 
      // returns a vector of which vehicle index should go to a specific goal
      std::vector<unsigned int> GoalVehicle;
   
    std::vector<double> distAssoc; // holds all distance values
      std::vector<double> timeAssoc; // holds all time values
      //since vehicles could drive at same time, accumulate time to test against that.
    std::vector<double> cumulativeTimeAssoc;
    pfms::nav_msgs::Odometry Blank; // This does nothing, just storage for an unneeded value
    
    std::vector<pfms::nav_msgs::Odometry> CurrentPos;

    for(int i = 0; i < controllers_.size();i++){
        CurrentPos.push_back(controllers_.at(i)->getOdometry()); // Store all the initial Odometries in here
        distAssoc.push_back(0); // Just used to initialize the vectors to the correct size
        timeAssoc.push_back(0);
        cumulativeTimeAssoc.push_back(0);
    }


        for(int i = 0; i < goals_.size(); i++){ //for each goal. compare distance/time for each vehicle to reach it. 
            double smallest_value =100000 ; // random high number used in checking for the smallest val;
            int Index = -1; 

            
            for(int j = 0; j < controllers_.size(); j++){
              


                if(controllers_.at(j)->checkOriginToDestination(CurrentPos.at(j),*goals_.at(i),distAssoc.at(j),timeAssoc.at(j),Blank)){
                if(objective_== mission::Objective::TIME){// if objective is equal to Time
                
                    if(timeAssoc.at(j)+cumulativeTimeAssoc.at(j)<smallest_value){
                        smallest_value = timeAssoc.at(j); //get smallest value
                        Index = j; //get index of smallest value

                       
                    }
                }
                else{
                   
                    if(distAssoc.at(j)<smallest_value){
                        smallest_value = distAssoc.at(j); // get smallest value
                        Index = j; // get index of smallest value
                    }
                }
                }
            }
            // Below Line pushes the estimated new position and time to travel of the robot to the current position
            
            if(Index != -1){ // At least 1 vehicle can reach it
            controllers_.at(Index)->checkOriginToDestination(CurrentPos.at(Index),*goals_.at(i),distAssoc.at(Index),cumulativeTimeAssoc.at(Index),CurrentPos.at(Index));
            GoalVehicle.push_back(Index);//if no vehicle can reach the goals, then it won't push back. This difference in length is caught and counts as a failure 
            }
        }
    return(GoalVehicle);
       


      
  }