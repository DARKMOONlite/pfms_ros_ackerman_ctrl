#include "quadcopter.h"
#include "ackerman.h"
#include "mission.h"
#include "controller.h"
#include "ackerman.cpp"
#include "quadcopter.cpp"
#include "controller.cpp"

#include <string>
#include <iostream>
#include <cmath>

int main(){
     //Function that takes input and then initialises either an Audi. Quadcopter or both
     unsigned int num_goals = 0;
    std::string Type;
    std::string MissionType;
    double GoalInput[4];


    std::vector<ControllerInterface*> Vehicles;


    while(1){ // if they haven't antered a correct sellection
    std::cout << "Enter desired system: Audi/Quadcopter/Both"<< std::endl;
    std::cin >> Type ;
    if(Type=="Audi"){
        
        Vehicles.push_back(new Ackerman());
        break;

    }
    else{
    if(Type=="Quadcopter"){
        
        Vehicles.push_back(new Quadcopter());
        break;

    }
    else{

    if(Type=="Both"){
        
        Vehicles.push_back(new Ackerman());
        Vehicles.push_back(new Quadcopter());
        break;
    }
    else{
        std::cout << "Unrecognised Input, enter again." << std::endl;
    }
    
    
    
    }
    }
    }
    Mission mission(Vehicles);
    while(1){
        while(1){
        //Ask for mission Type; either reach goal by shortest distance or shortest time between
        while(1){
            std::cout << "Mission Type: shortest distance or time, please enter (Distance/Time)" << std::endl;
            std::cin >> MissionType;
    
        
            if(MissionType == "Distance"){
                mission.setMissionObjective(mission::Objective::DISTANCE);
                break;
            }
            else{
                if(MissionType == "Time"){
                    mission.setMissionObjective(mission::Objective::TIME);
                    break;
                }
                else{
                    std::cout << "Invalid Mission Type, Try Again:" << std::endl;
                }
            }



        }
        //Ask the user for the location of 2 goals_
        
        std::cout << "Enter Goal Locations: X1" << std::endl;
        std::cin >> GoalInput[0];
        std::cout << "Enter Goal Locations: Y1" << std::endl;
        std::cin >> GoalInput[1];
        std::cout << "Enter Goal Locations: X2" << std::endl;
        std::cin >> GoalInput[2];
        std::cout << "Enter Goal Locations: Y2" << std::endl;
        std::cin >> GoalInput[3];
        std::vector<pfms::geometry_msgs::Point*> goals;

        std::cout << GoalInput[0] <<GoalInput[1] <<GoalInput[2] << GoalInput[4]<< std::endl;
        pfms::geometry_msgs::Point Goal1 = {.x = GoalInput[0],.y=GoalInput[1]};
        pfms::geometry_msgs::Point Goal2 = {.x = GoalInput[2],.y=GoalInput[3]};
                //goals.clear();
                goals.push_back(&Goal1);
                goals.push_back(&Goal2);
    
        
 

        mission.setGoals(goals);
    
       
        //Run the mission type

        bool succeeded = mission.runMission();
        //Repeat the last 3 steps
        if(succeeded){ // if all goals return positive 
            std::cout << "Goals Reached" << std::endl;
        }
        else{
            std::cout << "Goals Cannot be reached" << std::endl;
        }
        }
    }

}




