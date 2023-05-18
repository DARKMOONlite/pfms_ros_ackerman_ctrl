#include "randomarraygenerator.h"

RandomArrayGenerator::RandomArrayGenerator(int seed, int n):
    generator_(seed),
    distribution_(0,10),
    num_samples_(n)
{
}

void RandomArrayGenerator::populateArray(double array[])
{
    for (int i=0; i<num_samples_; i++) {
        array[i] = distribution_(generator_);
    }
}
