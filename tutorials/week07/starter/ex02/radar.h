#ifndef RADAR_H      // An 'include guard' to prevent double declaration of any identifiers in this library
#define RADAR_H

#include <string>
#include <vector>
#include <random>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
class Radar{
    public:

    Radar();
    ~Radar();
    std::vector<double> getData(void);	            // return a vector of targets, blocking call function
    double getScanningTime(void);                   // get Scanning Time
    void setScanningTime(double scanningTime);      // set Scanning Time
    void start(); 
    double getMaxDistance();

                     // start the thread that awaits for data

private:
    std::atomic<bool> running_;   
    void generateData(void);
    std::vector<double> data_;                 //targets as a vector
    double scanningTime_;
    const float maxDistance_ = 80.0;
    const unsigned int numTarget_ = 20;
    std::mt19937 *generator_;
    std::uniform_real_distribution<double> *value_;
    std::vector<std::thread> threads_;

    std::atomic<bool> ready_;
};
    std::mutex mtx_;
    std::condition_variable cv_;

#endif // SENSOR_H
