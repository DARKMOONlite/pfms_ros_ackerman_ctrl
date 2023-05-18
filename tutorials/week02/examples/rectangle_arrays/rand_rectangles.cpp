#include "rectangle.h"

#include <iostream>
#include <random>
#include <chrono>

double totalArea(Rectangle rectangles[], int size) {
    double total = 0.0;
    for (int i = 0; i < size; i++) {
        total += rectangles[i].area();
    }
    return total;
}

int main () {
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::uniform_real_distribution<double> dist(0,10);

    const int num_rectangles = 10;
    Rectangle rectangles[num_rectangles];

    // Note. Range based for loop only works directly on array
    // When arrays are passed to functions they decay to pointers
    // and this won't work!
    // Also note. The & before rectangle ensures we have a reference
    // instead of a copy so we can modify each rectangle
    for (auto &rectangle : rectangles) {
        rectangle.setWidthHeight(dist(gen), dist(gen));
    }

    std::cout << "Total area of all rectangles is " << totalArea(rectangles, num_rectangles) << std::endl;

    return 0;
}
