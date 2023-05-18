#include <iostream>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
using namespace std;

void sum_queue (std::queue<int>& num_queue,
                int& sum,
                std::mutex& mtx) {
                    std::unique_lock<std::mutex> lck(mtx);
                    
                    while(num_queue.size()>0){
                        lck.unlock();
                       int value = num_queue.front();
                        sum+=value;
                        num_queue.pop();
                        lck.lock();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }



                    
                    std::cout << "Thread sum is:" << sum << std::endl;

};




int main ()
{
    std::queue<int> input_data;
    std::mutex mtx;
    int sum1(0), sum2(0);
    for (int i = 0; i <500000;i++){
        input_data.push(i);
        input_data.push(-i);
    }
    std::thread sum_thread_1(sum_queue, ref(input_data), ref(sum1));
    std::thread sum_thread_2(sum_queue, ref(input_data), ref(sum2));
  

  sum_thread_1.join();
  sum_thread_2.join();

    return 0;
}
