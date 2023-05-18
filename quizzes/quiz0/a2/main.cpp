#include "sample.h"
#include <iostream>

int main () {


//  Create an object `sample` of `Sample` class
    Sample sample(5);
//  Display to standard output the value of parameter `value_` from the `sample` object.
    std::cout << sample.readValue() << std::endl;
    return 0;
}
