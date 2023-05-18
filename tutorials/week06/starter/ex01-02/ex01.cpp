#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

using namespace std;

void incrementNum(int &num, std::mutex &numMutex) {
    while (true) {
    //Use mutex 
    {
        std::unique_lock<std::mutex> lck(numMutex); //this locks between the bracket
        lck.unlock(); // this is usefull if you dont have a unique lock
        num++;
        lck.lock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    }
}

void printNum(int &num, std::mutex &numMutex) {
    while (true) {
    //Use mutex 

    {
       std:unique_lock<std::mutex> lck(numMutex); //this locks between the bracket
        std::cout << num << std::endl;
    }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


int main ()
{
    int num = 0;
    // We will use this mutex to synchonise access to num
    std::mutex numMutex;
 

    // Create the threads, how do we create the thread? What are the parameters
    std::thread inc_thread(incrementNum,ref(num),ref(numMutex));
    std::thread print_thread(printNum,ref(num),ref(numMutex));
    
    // Wait for the threads to finish (they wont)
    if(inc_thread.joinable()){
        inc_thread.join();
    }
     if(print_thread.joinable()){
    print_thread.join();
     }

    return 0;
}



