#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting
#include <condition_variable>
#include <vector>


class DataProcessing
{
public:
 DataProcessing();
 void generateSamples();
 void processSamples();

private:
  std::vector<double> data;
  // We will use this mutex to synchonise access to num
  std::mutex numMutex;

  std::condition_variable cv;

};

#endif // DATAPROCESSING_H
