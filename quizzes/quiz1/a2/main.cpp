#include <iostream>
#include "rectangle.h"
#include "processing.h"
#include "shape.h"


int main () {
    //! TODO: Create a rectangle
    Rectangle rectangle1;
    rectangle1.setHeightWidth(5.0, 6.0);

    // Print some info about it
    std::cout << "The area of " << rectangle1.getDescription() << " is " << rectangle1.getArea() << std::endl;
    std::cout << "It is a " << rectangle1.getDescription() << std::endl;



    Rectangle rectangle2(3,42);
    Rectangle rectangle3(6,2);
    Rectangle rectangle4(1,2);
    Rectangle rectangle5(5,4);
    Rectangle rectangle6(3,7);

    Rectangle *ptr1;
    Rectangle *ptr2;
    Rectangle *ptr3;
    Rectangle *ptr4;
    Rectangle *ptr5;
    Rectangle *ptr6;
    ptr1 = &rectangle1;
    ptr2 = &rectangle2;
    ptr3 = &rectangle3;
    ptr4 = &rectangle4;
    ptr5 = &rectangle5;
    ptr6 = &rectangle6;

    std::vector<Shape*> shapes;
    shapes.push_back(ptr1);
    shapes.push_back(ptr2);
    shapes.push_back(ptr3);
    shapes.push_back(ptr4);
    shapes.push_back(ptr5);
    shapes.push_back(ptr6);

    // for(int i = 0; i<shapes.size(); i++){
        
    //    shapes.at(i)->removeLargerThanArea(); 

    // }
    removeLargerThanArea(shapes,60);

    for(int i = 0; i<shapes.size(); i++){
        std::cout << "Shape at " << i << " has an area of: " << shapes.at(i)->getArea() << std::endl;
        
    }
}



