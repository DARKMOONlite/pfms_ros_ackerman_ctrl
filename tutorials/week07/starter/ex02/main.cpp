#include <iostream>
#include "radar.h"
#include "dataprocessing.h"
#include <thread>

int main (void){

  // instantiate radar object
  std::vector<Radar*> radars;

  // radars.push_back(new Radar);
  // radars.push_back(new Radar);
  // radars.push_back(new Radar);


  for(auto radar : radars){
    radar->start();
  }

std::shared_ptr<DataProcessing> dataProcessingPtr(new DataProcessing());

std::thread processing_thread(&DataProcessing::findClosestReading,dataProcessingPtr);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));
dataProcessingPtr->setRadars(radars);
  // while (true){
  //   std::vector <double> data = radar.getData();

  //   for(auto elem : data){
  //     std::cout << elem << " ";
  //   }
  //   std::cout << std::endl;
  // }

processing_thread.join(); // wait till processing thread is done

  for(auto radar : radars){
    delete radar;
  }

  return 0;
}

