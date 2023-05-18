#ifndef RANDOMVECTORFILLER_H
#define RANDOMVECTORFILLER_H

#include <vector>
#include <random>


class RandomVectorFiller
{
public:
    RandomVectorFiller(long seed, int n);
    void appendRandomNumbersTo(std::vector<double> &num_vec);

private:
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
    int count_to_append_;
};

#endif // RANDOMVECTORFILLER_H
