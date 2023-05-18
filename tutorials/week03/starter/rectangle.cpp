#include "rectangle.h"
#include "shape.h"

//    The `Rectangle` class needs a special member function.
//    Can you add to the `Rectangle` class function that enables the `Rectangle` to on creation have `width` , `height` and `description` initialised with values supplied by user of the class.


Rectangle::Rectangle(double width, double height){
    setHeightWidth(width,height);
    }

void Rectangle::setHeightWidth(double width, double height)
{
    //!NOTES
    // This is a example of why you should not allow direct access to member variables (why they are private)
    // Given we have a function to set the member varaibles, we also can leverage this function to set any
    // other member variables required, of perform any other operations that are needed to be executed
    // (such as invoking other methods)
    width_ = width;
    height_ = height;
    
    if (width_== height_) { //PROBING Q - Is this the best way to do floating point comparsion????
        description_ = "square";
    } else {
        description_ = "rectangle";
    }
}

double Rectangle::getArea(void)
{
    return width_ * height_;
}

