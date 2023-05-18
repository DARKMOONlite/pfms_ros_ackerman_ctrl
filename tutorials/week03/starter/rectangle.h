#ifndef RECTANGLE_H
#define RECTANGLE_H
#pragma once
#include "shape.h"

//#include <string>

//using std::string;
//Modify the file so Rectangle inherits from the base class of Shape

class Rectangle : public Shape //? This causes rectangle to take all the properties of Shape
{


public:
//    The `Rectangle` class is missing a special member function.
//    Can you add to the function that enables the `Rectangle` to on creation have `width` , `height` and `description` initialised with values supplied by user of the class.
    Rectangle(double width, double height);

    void setHeightWidth(double width, double height);
    // Consider if getArea() is a method that should exist in Rectangle?
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // A design to enable this is covered in when we introduce polymorphism
    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    double getArea (void);
private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?
    double width_; //!< width of rectangle
    double height_;//!< height of rectangle

};

#endif // RECTANGLE_H
