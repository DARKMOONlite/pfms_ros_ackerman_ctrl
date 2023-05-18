#include "circle.h"
#include <cmath>
#include <iostream>



// Circle::Circle(float x)
// {
//     radius_ = x;
// }

Circle::Circle(float x):
    radius_(x) // a different method to define the radius, this happens really quickly
{// try to put variable declarations before brackets
    radius_ = x; // and functions within
}

void Circle::setRadius(float x){
    radius_ = x;
}

float Circle::getArea(void){return(M_PI * radius_*radius_);
}

float Circle::getPerimeter(void){return(M_PI * radius_ * 2);}

void Circle::setPerimeter(float x){
    radius_ = x * M_1_PI * 0.5;
}
void Circle::setArea(float x){
    radius_ = sqrt(M_1_PI * x);
}

float Circle::print(void){
    return(radius_);
}
