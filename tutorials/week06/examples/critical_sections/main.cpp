#include <iostream>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>


void sum_queue (std::queue<int>& num_queue,
                int& sum,
                std::mutex& mtx) {

   mtx.lock();
   while (!num_queue.empty()) {
      sum += num_queue.front();
      num_queue.pop();
      mtx.unlock();
      
      mtx.lock();
   }
   mtx.unlock();
};

int main ()
{
    std::queue<int> input_data;
    std::mutex mtx;
    int sum1(0), sum2(0);

    for (int i = 1; i < 500001; i++) {
        input_data.push(i);
        input_data.push(-i);
    }

//OR
//    unsigned int seed = \
//      std::chrono::system_clock::now().time_since_epoch().count();

//    std::default_random_engine generator{seed};
//    std::normal_distribution<double> distribution{0, 20};

//    // Populate deque with guassian distributed data
//    for (unsigned int i = 0; i < num_values; i++)
//      values.push_back(distribution(generator));

    std::thread th1(sum_queue,
                    std::ref(input_data),
                    std::ref(sum1),
                    std::ref(mtx));
    std::thread th2(sum_queue,
                    std::ref(input_data),
                    std::ref(sum2),
                    std::ref(mtx));
    th1.join();
    th2.join();
    std::cout << "sum1=" << sum1 << ",  sum2=" << sum2 << ",  total=" << sum1+sum2 << std::endl;
    return 0;
}
