#include "quadcopter.h"
#include "ackerman.h"
#include "mission.h"
#include "controller.h"
#include "ackerman.cpp"
#include "quadcopter.cpp"
#include "controller.cpp"


#include "a1_types.h"
#include <iostream>
#include "logger.h"
#include "mission.h"
#include <string.h>

int main(int argc, char *argv[])
{

    //Default location of files containing sequence of points and objective
   std::string ackerman_filename = "../data/ACKERMAN.TXT";
   std::string quadcopter_filename = "../data/QUADCOPTER.TXT";
   mission::Objective objective =  mission::Objective::BASIC;

   // Will default to the settings unless specified with ovvride as arguments
   // Example ./assignment2 --advanced $HOME/data/ACKERMAN.TXT $HOME/data/QUADCOPTER.TXT
   // This woudl assume your files are in home directory in data subfolder
   // You do need the -- (two "-") to enable ADVANCED MODE
   if(argc<4){
        std::cout << "To override Run with 3 arguments: " << std::endl;
        std::cout << argv[0] << " MODE (--advanced) location_of_ACKERMAN.txt location_of_QUADCOPTER.txt" << std::endl;
    }
   else{    
       if(strcmp(argv[1],"-advanced")){
           std::cout << "Advanced Mode Activated" << std::endl;
           objective =  mission::Objective::ADVANCED;
       }
       ackerman_filename = argv[2];
       quadcopter_filename = argv[3];

   }

   std::vector<pfms::geometry_msgs::Point> ackermanPoints;
   std::vector<pfms::geometry_msgs::Point> quadcopterPoints;

   //If the files can not be opened we will terminate
    if(!logger::loadPoints(ackerman_filename,ackermanPoints)){
        std::cout << "Could not load points from file:" << ackerman_filename << std::endl;
        return 0;
    }

    if(!logger::loadPoints(quadcopter_filename,quadcopterPoints)){
        std::cout << "Could not load points from file:" << quadcopter_filename << std::endl;
        return 0;
    }




    std::vector<ControllerInterface*>Vehicles;
    Vehicles.push_back(new Quadcopter());
    Vehicles.push_back(new Ackerman());


    Mission mission(Vehicles);
    mission.setMissionObjective(objective);


    mission.setGoals(ackermanPoints,pfms::ACKERMAN);
    mission.setGoals(quadcopterPoints,pfms::QUADCOPTER);

    mission.run();

    bool OK = false;
    while (!OK){

        std::vector<unsigned int> progress = mission.status();

        int check = 0;
        for(int i =0; i <progress.size(); i++){
            std::cout<< "Progress: " << progress[i] << " " ;
            if(progress.at(i)==100){
                check++;
            }
        }
    std::cout << std::endl;
    if(check==progress.size()){
        std::cout << " All Vehicles reached Goals" << std::endl;
        break;
        }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    
}
return 0;
}
