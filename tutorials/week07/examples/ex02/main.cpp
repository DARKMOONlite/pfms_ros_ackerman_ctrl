#include <iostream>
#include <thread>
#include "dataprocessing.h"

int main ()
{

    //! The syntax to pass an member function argument to thraed constructor
    //! Requires two steps, creating a pointer (in this case a shared pointer)
    //! And thereafter passing the argument of member function and pointer
    //std::shared_ptr<Class> pointer(new Class(nh)); [Constructor is Class(nh)]

    std::shared_ptr<DataProcessing> dataProccessingPtr(new DataProcessing() );

    // Create the threads, now they run functions inside an object, so syntax is
    //std::thread t(&Class::function,pointerClass);
    std::thread inc_thread(&DataProcessing::generateSamples, dataProccessingPtr);
    std::thread print_thread(&DataProcessing::processSamples, dataProccessingPtr);

    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}



