#include "rectangle.h"

Rectangle::Rectangle():
    Rectangle(0.0, 0.0)
{
}

Rectangle::Rectangle(double width, double height):
    width_(width), height_(height)
{
}

void Rectangle::setWidthHeight(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Rectangle::area()
{
    return width_ * height_;
}

double Rectangle::perimeter()
{
    return 2 * (width_ + height_);
}


