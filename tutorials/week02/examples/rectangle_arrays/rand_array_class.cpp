#include "randomarraygenerator.h"
#include <iostream>
#include <chrono>

using std::cout;
using std::endl;

int main () {
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();

    // We set the number of samples to generate in the constructor of RandomArrayGenerator
    RandomArrayGenerator array_gen(seed, 10);

    // Here we allocate an array of a certain size
    double array[10];

    // Here we trust that the array size and number of samples
    // produced by RandomArrayGenerator match, but what if they don't?
    // There are two possibilities... How can we improve this?
    array_gen.populateArray(array);

    // Display the contents of the array
    cout << "array:[ ";
    for (auto value : array) {
        cout << value << " ";
    }
    cout << "]" << endl;
    return 0;
}
