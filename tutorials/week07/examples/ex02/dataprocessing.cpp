#include "dataprocessing.h"
#include <thread>
#include <iostream>

using std::mutex;
using std::vector;


DataProcessing::DataProcessing()
{

}

void DataProcessing::generateSamples() {

  //Setup and seed our random normal distribution generator
  std::default_random_engine generator(std::chrono::duration_cast
                                       <std::chrono::nanoseconds>
                                       (std::chrono::system_clock::now().time_since_epoch()).count());
  std::normal_distribution<double> distribution(6.0, 5.0); //mean of 6m and a stdev of 5m

  while (true) {

        // This delay is included to improve the emulate some other process of generating the data
        // by the sensor which could be at a specific rate
        std::this_thread::sleep_for (std::chrono::milliseconds(1000));


        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        std::unique_lock<std::mutex> lck(numMutex);

        std::cout << "sample gen" << std::endl;
        // We only access num while the mutex is locked
        double sample = distribution(generator);
        data.push_back(sample);

        numMutex.unlock();
        cv.notify_all();
    }
}

// This function consumes the samples
void DataProcessing::processSamples() {
    while (true) {
        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        std::unique_lock<std::mutex> lck(numMutex);

        //! http://www.cplusplus.com/reference/condition_variable/condition_variable/wait/
        //! The syntax is generally a lock followd by a predicate
        //! void wait (unique_lock<mutex>& lck, Predicate pred);
        //!
        //! lck
        //! A unique_lock object whose mutex object is currently locked by this thread.
        //!
        //! pred
        //! Needs to be a function (you would either need to make a function OR use a lambda function
        //!
        //! We use here a lambda function, which is a inline function (the function does not have a name and follows
        //! in curly brackets here
        //! the syntax for is [&] and then a function in {}

        cv.wait(lck, [&]{return !data.empty();});

        double sample = data.back();
        data.pop_back();
        lck.unlock();
        // We now have a sample
        std::cout <<  "sample is:" << sample << std::endl;
    }
}
