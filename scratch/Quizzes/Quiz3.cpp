#include <iostream>
#include <chrono>
#include <vector>
using namespace std::chrono;

class Timer 
{
private:
    time_point<high_resolution_clock> _startTime;

public:
    Timer() 
        : _startTime{ high_resolution_clock::now() }
    {}
    ~Timer() {  Stop(); }
    void Stop() 
    {
        const auto endTime = high_resolution_clock::now();
        const auto start = time_point_cast<microseconds>(_startTime).time_since_epoch();
        const auto end = time_point_cast<microseconds>(endTime).time_since_epoch();
        const auto durationTaken = end - start;
        const auto duration_ms = durationTaken * 0.001;
        std::cout << durationTaken.count() << "us (" << duration_ms.count() << "ms)\n";
    }
};
int main()
{
     std::vector<int> vec;
     vec.reserve(100);

     Timer time{};
     for (auto i=0; i < 1000000; ++i){
        vec.push_back(i);
     }
     std::cout << vec.size() << std::endl;
     return 0;
}