// We need to include the declaration of our new rectangle class in order to use it.
#include "rectangle.h"
#include <iostream>

int main () {

    // Create two rectangle objects
    Rectangle rectangle, square;

    // Set the values of the sides
    rectangle.setWidthHeight(5.0, 2.0);
    square.setWidthHeight(4.0);

    // Get the areas and print to screen
    double result = rectangle.area();
    std::cout << "rectangle area is " << result << std::endl;
    result = square.area();
    std::cout << "square area is " << result << std::endl;

    return 0;
}
