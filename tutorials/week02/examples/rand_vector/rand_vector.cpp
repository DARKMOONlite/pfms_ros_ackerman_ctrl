#include <iostream>
#include <vector>
#include <chrono>
#include "randomvectorfiller.h"

void printNumberVector(std::vector<double> numbers) {
    if (numbers.empty()) {
        std::cout << "Empty";
    } else {
        for (auto x : numbers) {
            std::cout << x << ' ';
        }
    }
    std::cout << std::endl;
}

int main() {
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    int count_to_append = 10;
    RandomVectorFiller filler(seed, count_to_append);

    std::vector<double> numbers;

    std::cout << "Numbers before: " << std::endl;
    printNumberVector(numbers);

    filler.appendRandomNumbersTo(numbers);

    std::cout << "Numbers after: " << std::endl;
    printNumberVector(numbers);

    filler.appendRandomNumbersTo(numbers);

    std::cout << "Numbers after again: " << std::endl;
    printNumberVector(numbers);

    return 0;
}
