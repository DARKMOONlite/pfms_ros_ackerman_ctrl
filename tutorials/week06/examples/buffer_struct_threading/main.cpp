#include <iostream>
#include <thread>
#include <chrono>
#include <random>

#include "databuffer.h"

void addNumber(DataBuffer &buffer) {
    // Init random number generation
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0,100);

    while (true) {
        buffer.buffer_mutex_.lock();
        buffer.values.push_back(distribution(generator));
        cout << "Added value: " << buffer.values.back() << endl;
        buffer.buffer_mutex_.unlock();
        // This delay slows the loop down for the sake of readability
        //std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
}

void removeValues(DataBuffer &buffer, double min, double max) {
    while (true) {
        buffer.buffer_mutex_.lock();

        auto it = buffer.values.begin();

        while ( it != buffer.values.end()) {
            if (*it < min || *it > max) {
                buffer.values.erase(it);
                cout << "Erased value: " << *it << endl;
            } else {
                it++;
            }
        }
        buffer.buffer_mutex_.unlock();
        // This short delay prevents this thread from hard-looping and consuming too much cpu time
        // Using a condition_variable to make the thread wait provides a better solution to this problem
        //std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
}

void trimLength(DataBuffer &buffer) {
    while (true) {
        buffer.buffer_mutex_.lock();

        while (buffer.values.size()>20) {
            cout << "Size is " << buffer.values.size() << " removing first value" << endl;
            buffer.values.erase(buffer.values.begin());
        }
        buffer.buffer_mutex_.unlock();
        // This short delay prevents this thread from hard-looping and consuming too much cpu time
        // Using a condition_variable to make the thread wait provides a better solution to this problem
        //std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
}



int main ()
{
    // Create the shared buffer which contains its own mutex
    DataBuffer data_buffer;

    // Start all the threads
    thread add_number_thread(addNumber,std::ref(data_buffer));
    thread remove_values_thread(removeValues,std::ref(data_buffer),20,80);
    thread trim_length_thread(trimLength,std::ref(data_buffer));

    // Wait for the threads to finish (they wont)
    add_number_thread.join();
    remove_values_thread.join();
    trim_length_thread.join();

    return 0;
}



