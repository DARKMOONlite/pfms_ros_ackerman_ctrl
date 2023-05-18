#include "rectangle.h"

//    TASK 2
//    The `Rectangle` class already has a special member function.
//    Thanks to *Polymorphism* we can have functions with same name but different number of parameters.
//    Can you add to the `Rectangle` class another function that enables the `Rectangle` to on creation have `width` , `height` and `description` initialised with values supplied by user of the class.
//    You will need to add the declaration of this member function in the [header of Rectangle class](./a2/rectangle.h) as
//    well as implement this function in [implementation file of Rectangle class](./a2/rectangle.cpp).

Rectangle::Rectangle(){}

Rectangle::Rectangle(double width, double height): width_(width), height_(height) //initialisation list
{
    if(width_ == height_){
        description_="square";
    }
    else{description_="rectangle";}

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



// void Rectangle::removeLargerThanArea(std::vector<Shape*> &shapes, double limit){
//     for(int i=0; i<shapes.size(); i++){
//         if(shapes.at(i)->getArea() > limit){
//             shapes.erase(shapes.begin()+i); //this is a weird workaround because erase requires an iterator.
//             i--;          //.at doesn't give itterator but .begin does
//         }
//     }

//}


