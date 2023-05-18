#include "container_ops.h"
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers


//!TODO TASK 1:
//! Implement function that accepts a container and modifies it by adding user specified numbers of elements
//! To front of container. The actual element is also supplied by user (for instance num_values =4 element =-1.5  means
//! four elements of -1.5 are added to begining of deque)
void populateContainer(std::deque<double>& container, unsigned int num_values, double element){
    

    for(int i = 0; i < num_values; i++){
        container.push_front(element);

    }

}

//!TODO TASK 2: Implement function that accept the chosen container and rearranges elements by bubble sort operation
//! An example on arrays with C++ code is here https://www.programiz.com/dsa/bubble-sort
void bubbleSortContainer( std::deque<double>& container){
    for(int i = 0; i < container.size()-1;i++){ //go through the whole container
        for(int j = 0; j<container.size()-1-i; j++)       //for each element go from there to the end.  
            {
            if(container.at(j) > container.at(j+1)){
                std::swap(container.at(j), container.at(j+1));
            }

            }


    }
    

}


