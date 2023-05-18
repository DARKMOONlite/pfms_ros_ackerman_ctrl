#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting
#include <condition_variable> // condition variable

using namespace std;


// The function generates samples
void generateSamples(vector<double> &data, mutex &numMutex, std::condition_variable &cv) {

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
        // is not locked anywhere else, we need unique lock for conditional variable to work
        std::unique_lock<std::mutex> lck(numMutex);

        std::cout << "sample gen" << std::endl;
        // We only access num while the mutex is locked
        double sample = distribution(generator);
        data.push_back(sample);

        //! The process now of synchronising access (waking up other thraeds waiting for data) now involves
        //! 1. Unlocking the mutex, so other threads could get hold if it
        //! 2. Calling notify_all (will notify all thraeds waiting for data
        numMutex.unlock();
        cv.notify_all();

    }
}

// This function consumes the samples
void processSamples(vector<double> &data, mutex &numMutex, std::condition_variable &cv) {
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
      //! We use here a lambda function, which is like an inline function (the function does not have a name and follows in curly brackets here
      //! https://docs.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-160
      //!
      //! A lambda begins with the capture clause. It specifies which variables are captured, and whether the capture is by value
      //! or by reference. Variables that have the ampersand (&) prefix are accessed by reference and variables that don't have it are accessed by value
      //! An empty capture clause, [ ], indicates that the body of the lambda expression accesses no variables in the enclosing scope.
      //! [&] means all variables that you refer to are captured by reference,
      //cv.wait(lck, [&]{return !data.empty();});
      cv.wait(lck, [&data]{return !data.empty();});




      double sample = data.back();
      data.pop_back();
      lck.unlock();
      // We now have a sample
      std::cout <<  "sample is:" << sample << std::endl;

      //! The last question in tutorial, suggested we should look at writing code to remove sample closest to the mean
      //! Does tghis make sense in the way the threads are working? How many ssamples would be in the vector at any one time
      //! with this type of sync
    }
}

int main ()
{
    vector<double> data;
    // We will use this mutex to synchonise access to num
    mutex numMutex;
    // We now need to share a conditional variable as well, to achive sharing data between threads
    std::condition_variable cv;

    // Create the threads
    thread inc_thread(generateSamples,ref(data),ref(numMutex), ref(cv));
    thread print_thread(processSamples,ref(data),ref(numMutex), ref(cv));
 
    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}



