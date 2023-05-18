#include "shape.h"
#include<vector>

Shape::Shape():
    description_("unknown shape")
{
}

void Shape::setCentre(double x, double y)
{
    centreX_=x;
    centreY_=y;
}


std::string Shape::getDescription()
{
    return description_;
}

double Shape::getArea (void){
    return(0);
}

