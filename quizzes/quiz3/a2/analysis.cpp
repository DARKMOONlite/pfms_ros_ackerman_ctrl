#include "analysis.h"

#include <iostream> // Only here for showing the code is working
#include <thread>

using std::vector;
Analysis::Analysis(std::shared_ptr<Radar> radarPtr):
    radarPtr_(radarPtr)
{

}

//! @todo
//! TASK 1 and 2 - Same implementation, just being called twice Refer to README.md and the Header file for full description
void Analysis::computeScanningSpeed(unsigned int samples, double& scanningSpeed){
  auto start = std::chrono::steady_clock::now();
  
  vector<double> results ; 
  for (unsigned int i = 0; i < samples; i++){
  results = radarPtr_->getData();

  }
  auto end = std::chrono::steady_clock::now();

  // for(int i = 0; i < results.size(); ++i){
  //   std::cout << results.at(i)<< std::endl;
  // }
  std::chrono::duration<double> diff = end - start;
  //std::cout << "Time Taken: " << diff.count() << std::endl;

  scanningSpeed = diff.count()/samples;

  return;
}
