#ifndef RANDOMARRAYGENERATOR_H
#define RANDOMARRAYGENERATOR_H

#include <random> // Includes the random number generator

class RandomArrayGenerator
{
public:
    RandomArrayGenerator(int seed, int n);
    void populateArray(double array[]);

private:
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> distribution_;
    int num_samples_;
};

#endif // RANDOMARRAYGENERATOR_H
