// We need to include the declaration of our new circle class in order to use it.

#include <iostream>
#include <vector>
#include "circle.h"
#include "circle.cpp"
using std::vector;




int main () {
    float x = 1, y = 2, z = 5;
    Circle Circle_1(x);
    Circle Circle_2(y);
    Circle Circle_3(z);

    vector<Circle> circles;
    circles.push_back(Circle(1.0));
    circles.push_back(Circle(2.0));
    circles.push_back(Circle(3.0));

    for(int i = 0; i < circles.size(); i++){
        std::cout << circles.at(i).getArea() << std::endl;
    }

    for (auto circle:circles){ //auto takes 1 element from circles which is a circle
        std::cout << circle.getArea() << std::endl;
    } //? This does the same as the for loop above


    //std::cout << Circle_1.getArea() << std::endl;
    //std::cout << Circle_2.getPerimeter() << std::endl;
    return 0;
}

