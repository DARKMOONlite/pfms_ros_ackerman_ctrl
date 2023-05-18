#include <iostream>
#include "rectangle.h"
#include <vector>
//#include <string>
#include "shape.h"
//#include "rectangle.cpp"
//#include "shape.cpp"

int main () {

    //! TODO: Create a rectangle
    Rectangle rectangle1(1,1);
    rectangle1.setHeightWidth(5.0, 3.55);
   

    std::cout << "The area of " << rectangle1.getDescription() << " is " << rectangle1.getArea() << std::endl;
    std::cout << "It is a " << rectangle1.getDescription() << std::endl;



//std::vector<Shape*> shapes;
//shapes.push_back(new Rectangle(5,4));




}


