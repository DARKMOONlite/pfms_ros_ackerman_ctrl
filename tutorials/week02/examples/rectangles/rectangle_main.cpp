// We need to include the declaration of our new rectangle class in order to use it.
#include "rectangle.h"

#include <iostream>
#include <vector>
#include <random>
#include <chrono>

// Defining functions like this above main is not best practice
// We will talk later about where this function belongs
double computeTotalArea(std::vector<Rectangle> rectangles) {
    double total_area = 0;
    for (auto r : rectangles) {
        total_area += r.area();
    }
    return total_area;
}

int main () {

    // Create an empty vector of rectangles
    std::vector<Rectangle> rectangles;

    // Setup random number generation using seed from system clock
    int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::uniform_real_distribution<double> dist(1.0, 10.0);

    std::cout << "Area before: " << computeTotalArea(rectangles) << std::endl;

    int num_rectangles = 10;
    // Loop through and add a bunch of rectangles with random (1.0-10.0) sides
    for (int i = 0; i < num_rectangles; i++) {
        rectangles.push_back(Rectangle(dist(gen), dist(gen)));
    }
    // Compute the total area and print it
    double total_area = computeTotalArea(rectangles);
    std::cout << "Area after: " << total_area << std::endl;

    // Add yet more random rectangles
    for (int i = 0; i < num_rectangles; i++) {
        rectangles.push_back(Rectangle(dist(gen), dist(gen)));
    }
    total_area = computeTotalArea(rectangles);
    std::cout << "Area after again: " << total_area << std::endl;

    return 0;
}
