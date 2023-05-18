#include "arrays.h"
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <iostream>

// function to populate array with random numbers
void populateWithRandomNumbers(double x[], unsigned int& array_size, unsigned int num_elements) {

    //we select a seed for the random generator, so it is truly random (never the same seed)
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a normal distribution with means zero and standard deviation 10
    std::normal_distribution<> distribution(0,10.0);
    // generate the required amount of random numbers
    for (unsigned int i=array_size; i<array_size+num_elements; i++) {
        x[i] = distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}


//1) TASK: Implement function `assignArrayToVector`that assigns elements of array `x` to a vector named `vec`
void assignArrayToVector(double x[] ,unsigned int arraySize ,std::vector<double> &myVec ){
    for(unsigned int i=0; i<arraySize; i++){
        myVec.push_back(x[i]);
    }
}



//2) TASK: Implement function `removeNumbersLargerThan` that accepts vector `myVec` and removes elements of `myVec` greater than a `limit`
void removeNumbersLargerThan(std::vector<double> &myVec, double limit){
    for (unsigned int i = 0; i<myVec.size(); i++){
        if(myVec.at(i)>limit){
            myVec.erase(myVec.begin()+i);
            i--;
        }
    }
}


//3) TASK: Implement function `computeMeanAndStdDev` that computes the mean and standard deviation of `myVec` and returns the `Stats` structure with these values.
Stats computeMeanAndStdDev(std::vector<double> myVec){
    double mean = 0;
    for(int i = 0; i<myVec.size(); i++){
        mean += myVec.at(i);
    }
    mean /= myVec.size();
    double sum = 0;
    for(int i = 0; i<myVec.size(); i++){
        sum += pow((myVec.at(i)-mean),2);
    }
    double st_dev = pow(sum/myVec.size(),0.5);
    Stats stats;
    stats.mean = mean;
    stats.std_dev = st_dev;
    return(stats);
}

//4) TASK:  Implement function `returnVecWithNumbersSmallerThan` that returns a vector containing elements of `myVec` vector that are less than `limit`.
std::vector<double> returnVecWithNumbersSmallerThan(std::vector<double> myVec, double limit){

    //BONUS - Can you use a lambda function instead of looping?
    std::vector<double> smallVec;
    
    for(double x : myVec){
        if(x<limit){smallVec.push_back(x);}
    }
    return(smallVec);
}

