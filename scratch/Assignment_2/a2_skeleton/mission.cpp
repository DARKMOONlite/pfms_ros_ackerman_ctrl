
#include <vector>
#include "mission.h"
#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <queue>
#include <set>
#include <algorithm>
#include <iostream>
using pfms::geometry_msgs::Goal;
using pfms::geometry_msgs::Point;

Mission::Mission(std::vector<ControllerInterface*> controllers){
controllers_ = controllers;

std::mutex mtx;
std::condition_variable cv;

Total_D.assign(controllers_.size(),0);

}



bool Mission::run(){




std::cout << "num Controllers: "<< controllers_.size() <<std::endl;
std::cout << "number of goals" << goals_.size() << std::endl;
    switch(objective_){
        case mission::BASIC:
        for(int i=0; i<controllers_.size(); i++){ //! Maybe         move the passing of goals to set goal rather than run for unit testing purposes
            ordered_goals.push_back(BasicMethod(goals_,i));
        }
            break;

        case mission::ADVANCED:
        for(int i=0; i<controllers_.size(); i++){
            ordered_goals.push_back(FindBestPath(i,DorT));
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
        }
        break;
        default:
            std::cout << "no correct mission given" << std::endl;
            return 0;
  
    }
    for(int i = 0; i < ordered_goals.at(0).size(); i++){
        std::cout << ordered_goals.at(0).at(i).x << " "<< ordered_goals.at(0).at(i).y << std::endl;
    }


    for(int i=0; i<controllers_.size(); i++){
        controllers_.at(i)->setGoals(ordered_goals.at(i));
    }
     
    Total_D = Max_Distance();
    std::cout << objective_ << std::endl;
    std::cout << "Max Distance Size" << Total_D.size()<< std::endl;
    std::cout << "Max Dist First Val:" <<Total_D.at(0)<< std::endl;
  
    //! Create Threads for each of the run functions
    for(auto vehicle : controllers_){
        vehicle->run();
    }


    //Create thread for status and make it post percentage distance traveled every 5s

    //in main thread run the vehicles around the goals.
    return(1);

}

std::vector<unsigned int> Mission::status(void){ 
   
    
    vector<unsigned int> status;
   
    for(int i=0; i< controllers_.size(); i++){
       // std::cout << controllers_.at(i)->distanceTravelled()/Total_D.at(i);
       
       unsigned int percentage = (unsigned int)(controllers_.at(i)->distanceTravelled()/Total_D.at(i)*100);
       if(percentage >90  && controllers_.at(i)->status() == pfms::IDLE){percentage =100;}
       else{if(percentage>100 && percentage <150){percentage =99;}}
    //    std::cout << "Percentage: " << percentage << std::endl;
       
       status.push_back(percentage);
      
    }



    
   return(status);


    

}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform){
    //sets goals to all controllers
    
    goals_ = goals;

    goal_pipes_ = new Pipes(5,"/goal_buffer_seg","/goal_buffer_wsem","/goal_buffer_rsem");
    
    for(int i = 0; i < goals.size(); i++){ // for each goal in the vector

    // send the goals to the pipe to visualise them
    Goal goal {static_cast<unsigned long>(i),
        {goals.at(i).x,goals.at(i).y} //<seq_num> <goal_1_x> <goal_1_y>
         };


    //goal_pipes_.writeCommand(goal);
    goal_pipes_->writeCommand(goal);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    }
    
    // for(int i = 0; i <controllers_.size(); i++){
    //     if(controllers_.at(i)->getPlatformType()==platform){
    //     controllers_.at(i)->setGoals(goals);
    //     }
    // }

    return;

}

//   bool Mission::runMission(){ //Runs the mission

//       std::vector<unsigned int> GoalAssoc = getPlatformGoalAssociation(); //Take each goal and give them to either the closest vehicle by time or distance
//       bool Possible;
//       //GoalAssoc is a vector containing which vehicle should go to each goal
//       if(GoalAssoc.size() != goals_.size()){return false; //Catches cases where no vehicle can reach a goal
//       }
//       for(unsigned int i = 0; i < GoalAssoc.size();i++){

//           Possible = controllers_.at(GoalAssoc.at(i))->setGoal(*goals_.at(i)); // set the goals
//             if(Possible==0){
//                 return(false);
//             }
//           controllers_.at(GoalAssoc.at(i))->reachGoal();
          
//       }
//       return(true);

//   }

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
          auto Time = controllers_.at(i)->timeTravelled();
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
              


                if(controllers_.at(j)->checkOriginToDestination(CurrentPos.at(j),goals_.at(i),distAssoc.at(j),timeAssoc.at(j),Blank)){
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
            controllers_.at(Index)->checkOriginToDestination(CurrentPos.at(Index),goals_.at(i),distAssoc.at(Index),cumulativeTimeAssoc.at(Index),CurrentPos.at(Index));
            GoalVehicle.push_back(Index);//if no vehicle can reach the goals, then it won't push back. This difference in length is caught and counts as a failure 
            }
        }
    return(GoalVehicle);
       


      
  }


  std::vector<Point> Mission::FindBestPath(int Controller_Index,bool DorT){
    int num = 0;
    int num2 = 0;
    Mission::path_pos cur_path_pos;

    cur_path_pos.odos.push_back(controllers_.at(Controller_Index)->getOdometry()) ;
    cur_path_pos.distances.push_back(0);
    std::vector<double> distances;
    std::deque<int> visited;
    std::deque<int> Cur_Short_Path;
    std::vector<double> cur_length (goals_.size(),0);
    std::vector<int> indexes;
    std::vector<std::set<int>> Keys(goals_.size()+1); //keys should be 1 bigger. but it is unused
    //fill(Keys.begin(), Keys.end(),std::set<int>());
    for(int i =0; i< goals_.size(); i++){
        indexes.push_back(i);
        
        }
  
    auto min_path = std::numeric_limits<double>::max();
    auto start = std::chrono::system_clock::now();

    do{
  
        for(int i = 0; i< goals_.size(); i++){ //This checks if each goal has already been reached or checked already
        
   
          
                





            if(std::find(visited.begin(), visited.end(),i) == visited.end() && Keys.at(visited.size()).find(i) == Keys.at(visited.size()).end()){ // If the goal hasn't already been visited 
                Keys.at(visited.size()).insert(i);
                visited.push_back(i);
                Keys.at(visited.size()).clear();
                
                cur_path_pos =  Distance_through(cur_path_pos,goals_.at(i),Controller_Index,0);
                i=0;
                if(cur_path_pos.distances.back()<0){
                        visited.pop_back();
                        cur_path_pos.distances.pop_back();
                        //cur_path_pos.odos.pop_back(); //? done need to pop back odos in thiss case because we dont set a new odo
                        num++;
                        
                        break;
                }


                if(visited.size() != goals_.size()){
                     // since we just added to stack this clears the memory of the level we are just entering. thus 
                    
                    if(cur_path_pos.total_dist> min_path){
                       // std::cout <<"poping because path too long " << visited.size() << std::endl;
                        visited.pop_back();
                        cur_path_pos.distances.pop_back();
                        cur_path_pos.odos.pop_back();
                       
                        num++;
                    }
                
                }
                
                //calculate distance between
                //if distance is greater than min_total distance then pop
                else{ // if this is the final goal in the list then
                                //?cur_path = DistanceThroughPath(visited,Controller_Index,min_path);
                                
               
                    //?check total distance, if smaller than current min distance, then update.
                
                 if(cur_path_pos.total_dist < min_path){ 
                     if(visited.size()==goals_.size()){
                         min_path = cur_path_pos.total_dist;
                         Cur_Short_Path = visited;
                         distances = cur_path_pos.distances;
                     }

                         
                     
                 }

                break; // break out of for lo   op
            }
            }
            else{
                if(i == goals_.size()-1){ // if it reaches this point. it means that all goals in the next step are either on the stack. or already visited.

      
                    visited.pop_back();
                    cur_path_pos.distances.pop_back();
                    cur_path_pos.odos.pop_back();
                    
                    num++;
                    
                }
            }

        }
        
        for(int j = 0; j < visited.size(); j++){
            std::cout << visited.at(j) << " ";
        }
        std::cout << std::endl;

       
    }while(visited.size()!= 0 || Keys.at(0).size() != goals_.size() );
        
        auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed = end-start;
        std::cout << "time taken to calculate " << goals_.size() << " goals: " << elapsed.count() << std::endl;
        // std::cout << "number of paths: " << num << std::endl;
        std::cout << "fastest path " << min_path << std::endl;
  
        Total_D.at(Controller_Index)= min_path; // stores the path distance for later use
        std::vector<Point> BestPath;
        for(int i = 0; i < Cur_Short_Path.size(); i++){
        //std::cout << "Pushing:"<<Cur_Short_Path[i] << "   " << distances[i] << std::endl; 
            BestPath.push_back(goals_.at(Cur_Short_Path.at(i)));
        }

         
        return(BestPath);

    //! Other model has been moved to the Test folder in scratch.
}
  

double Mission::DistanceThroughPath( deque<int> path,int Controller_Index,double cur_min_dist){ //! needs to be fleshed out once ackerman and quadcopter are cleaned.
    double total_distance = 0;
    pfms::nav_msgs::Odometry predicted_odo;
    double predicted_dist = 0;
    double predicted_time = 0;
    
    bool reached = 0;

    for (int i = 0; i < path.size(); i++){
        //std::cout << i << std::endl;
        //std::cout <<"path size:"<< path.size() << std::endl;
        if(i==0){
            reached = controllers_.at(Controller_Index)->checkOriginToDestination(controllers_.at(Controller_Index)->getOdometry(),goals_.at(path.at(i)),predicted_dist,predicted_time,predicted_odo);
            
            }

        else{
            reached = controllers_.at(Controller_Index)->checkOriginToDestination(predicted_odo,goals_.at(path.at(i)),predicted_dist,predicted_time,predicted_odo);
            }
            //std::cout <<"reached:"<< reached << std::endl;

        //std::cout << total_distance << std::endl;
        total_distance += predicted_dist; // find the distance for each section and add them up

        if(total_distance > cur_min_dist){return(total_distance);}
        if(reached==0){
            return(-1);
        }
    }
   

//virtual LinkedOdometry DistanceFromTo(pfms::nav_msgs::Odometry& Odo, Goal goal)=0;
    return(total_distance);
}



Mission::path_pos Mission::Distance_through(Mission::path_pos cur_pos,pfms::geometry_msgs::Point cur_goal,int Controller_Index, bool DorT){
bool reached = 0;
    double predicted_dist = 0;
    double predicted_time = 0;
    pfms::nav_msgs::Odometry predicted_odo;
    Mission::path_pos newOdo;
    newOdo.total_dist = 0;
    
  
    
  
  reached = controllers_.at(Controller_Index)->checkOriginToDestination(cur_pos.odos.back(),cur_goal,predicted_dist,predicted_time,predicted_odo);
  
    if(reached==0){
        newOdo = cur_pos;
        newOdo.distances.push_back(-1);
       
    }
    else{
        if(DorT == 0){
            newOdo.distances = cur_pos.distances;
            newOdo.distances.push_back(predicted_dist);
            newOdo.odos = cur_pos.odos;
            newOdo.odos.push_back(predicted_odo);

            for(int i=0; i <newOdo.distances.size(); i++){
                newOdo.total_dist+=newOdo.distances.at(i);
                }
                }
        else{
              newOdo.distances = cur_pos.distances;
            newOdo.distances.push_back(predicted_time);
            newOdo.odos = cur_pos.odos;
            newOdo.odos.push_back(predicted_odo);

            for(int i=0; i <newOdo.distances.size(); i++){
                newOdo.total_dist+=newOdo.distances.at(i);
            }
          }
    //std::cout << "reached" << reached<< std::endl;
    //std::cout << "math Distance" <<newOdo.distances.back()<<std::endl;

    }
    
return(newOdo);

}



vector<double> Mission::Max_Distance(){
    vector<double> Total_Distances;
    if(ordered_goals.size()==0){
        std::cout << "Ordered Goals not set" << std::endl;
        
    }

    for(int i =0; i<controllers_.size(); i++){
        double total_distance = 0;
        pfms::nav_msgs::Odometry predicted_odo;
        double predicted_dist = 0;
        double predicted_time = 0;
    
        bool reached = 0;
        for (int j = 0; j < ordered_goals.at(i).size(); j++){
        //std::cout << i << std::endl;
        //std::cout <<"path size:"<< path.size() << std::endl;
        if(j==0){
            reached = controllers_.at(i)->checkOriginToDestination(controllers_.at(i)->getOdometry(),ordered_goals.at(i).at(j),predicted_dist,predicted_time,predicted_odo);
            
            }

        else{
            reached = controllers_.at(i)->checkOriginToDestination(predicted_odo,ordered_goals.at(i).at(j),predicted_dist,predicted_time,predicted_odo);
            }
        total_distance += predicted_dist;



    }
    Total_Distances.push_back(total_distance);
}
return(Total_Distances);
}

 std::vector<Point> Mission::BasicMethod(std::vector<Point> path, int Controller_Index){
     std::vector<Point> result;
     int num = 0;
     pfms::nav_msgs::Odometry predicted_odo = controllers_.at(Controller_Index)->getOdometry();
     double predicted_dist;
     double predicted_time;
     for(int i = 0; i < path.size(); i++){
         bool reached = controllers_.at(Controller_Index)->checkOriginToDestination(predicted_odo,path.at(i),predicted_dist,predicted_time,predicted_odo);
         if(reached != 0){
             result.push_back(path.at(i));
         }
         else{
             num++;
         }
     }
    std::cout << "removed " <<num <<" unreachable goals" << std::endl;
    return(result);

 }
