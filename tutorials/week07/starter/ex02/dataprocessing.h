#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H
#include <vector>
#include "radar.h"
#include <iostream>

class DataProcessing
{
public:
  DataProcessing();
  void findClosestReading();
  void setRadars(std::vector<Radar*> radars);
  private:
  std::vector<Radar*> radars_ ;
};


#endif // DATAPROCESSING_H


