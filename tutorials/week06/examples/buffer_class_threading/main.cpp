#include <iostream>
#include <thread>
#include <chrono>
#include <random>

#include "databuffer.h"

using namespace std;

// Refer below link for the [[noreturn]] specifier
// https://en.cppreference.com/w/cpp/language/attributes/noreturn
[[noreturn]] void addNumber(DataBuffer &buffer) {
    // Init random number generation
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0,100);

    while (true) {

        double value =distribution(generator);
        buffer.addValue(value);

        //cout << "Added value: " << value << endl;

        // This delay slows the loop down for the sake of readability
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

[[noreturn]] void removeValues(DataBuffer &buffer, double min, double max) {
    while (true) {
        buffer.removeValues(min, max);

        // This short delay prevents this thread from hard-looping and consuming too much cpu time
        // Using a condition_variable to make the thread wait provides a better solution to this problem
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

[[noreturn]] void trimLength(DataBuffer &buffer) {
    while (true) {
        buffer.trimLength(20);
        // This short delay prevents this thread from hard-looping and consuming too much cpu time
        // Using a condition_variable to make the thread wait provides a better solution to this problem
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}



int main ()
{
    // Create the shared buffer which contains its own mutex
    DataBuffer data_buffer;

    // Start all the threads
    thread add_number_thread(addNumber,ref(data_buffer));
    thread remove_values_thread(removeValues,ref(data_buffer),20,80);
    thread trim_length_thread(trimLength,ref(data_buffer));

    // Wait for the threads to finish (they wont)
    add_number_thread.join();
    remove_values_thread.join();
    trim_length_thread.join();

    return 0;
}



