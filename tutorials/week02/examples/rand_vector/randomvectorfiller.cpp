#include "randomvectorfiller.h"
#include <random>

RandomVectorFiller::RandomVectorFiller(long seed, int n):
    generator_(seed), distribution_(5.0, 2.0), count_to_append_(n)
{
}

void RandomVectorFiller::appendRandomNumbersTo(std::vector<double> &num_vec)
{
    for (int i = 0; i < count_to_append_; i++) {
        num_vec.push_back(distribution_(generator_));
    }
}
