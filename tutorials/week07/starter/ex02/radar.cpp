#include <iostream>
#include <stdlib.h>
#include "radar.h"
#include <vector>
#include <chrono>
#include <thread>


Radar::Radar():
  scanningTime_(100){

  //Generate first random value
  std::random_device rd;
 
  generator_= new std::mt19937(rd());
  value_ = new std::uniform_real_distribution<double>(0.1,80.0);

  data_.resize(numTarget_);
}

Radar::~Radar(){
  running_ = false;
  for(auto & t: threads_){ // & is needed for some reason
    t.join(); // Join waits for system to finish.
  }
}

void Radar::start(){
  running_ = true;
  
  threads_.push_back(std::thread(&Radar::generateData,this)); // need to put & when callingn a function within a class
  // also have to pass this if this thread is called within a class
}

void Radar::generateData(){
  while(running_){

  //generate random number of targets for each target (N) create Target containing random range and bearing between ^stored values
  // Info on dereferecing pointer https://stackoverflow.com/questions/27081035/why-should-i-dereference-a-pointer-before-calling-the-operator-in-c/27081074#27081074
  std::unique_lock<std::mutex> lck(mtx_); //locks Mutex
  for (unsigned int i=0; i < numTarget_; i++){
    data_.at(i)=value_->operator()(*generator_);
  }
  ready_ = true;
  lck.unlock(); //Unlocks mutex
  cv_.notify_all();
  
  if(!running_){break;} // check if we need to terminate before sleep

 
  std::this_thread::sleep_for (std::chrono::milliseconds(static_cast<int>(scanningTime_)));
 }
}


std::vector<double> Radar::getData(){
  std::unique_lock<std::mutex> lck(mtx_); //locks Mutex

  cv_.wait(lck,[&](){return ready_==true;});
  //or while(!ready_) cv_.wait(lck)
  std::vector<double> data = data_;
  ready_ = false;

  lck.unlock();
  //generateData();
  return data;
}


void Radar::setScanningTime(double scanningTime){
  scanningTime_ = scanningTime;
}

double Radar::getScanningTime(void){
  return scanningTime_;
}
double Radar::getMaxDistance(){
  return maxDistance_;
}


