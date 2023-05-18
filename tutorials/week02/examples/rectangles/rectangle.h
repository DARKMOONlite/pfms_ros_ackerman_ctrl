#ifndef RECTANGLE_H
#define RECTANGLE_H

// Declaration of the rectangle class
class Rectangle {

// Public members are accessible from outside the class (ie. in main)
public:
    // Declare the constructor
    Rectangle();
    Rectangle(double width, double height);
    // Declare the set_values method
    void setWidthHeight (double width, double height);
    // Declare the area method
    double area();
    //Declare perimeter method
    double perimeter();

// Private members are only accessible from within methods of the same class
private:
    // This class has two integers to represent the sides of the rectangle
    // The trailing underscore is used to differentiate the member varibles
    // ..from local varibles in our code, this is not compulsary
    double width_, height_;

};

#endif // RECTANGLE_H
