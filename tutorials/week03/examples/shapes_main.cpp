#include "circle.h"
#include "rectangle.h"
#include <iostream>
#include <vector>

using std::cout;
using std::endl;

int main () {

    Circle circle(3.0);//radius 3
    cout << "The area of circle is " << circle.getArea() << endl;
    cout << "It is a " << circle.getDescription() << endl;
    circle.setCentre(3.0,3.0);//Lets move centre to 3.0 3.0 so does not intercept
    cout << "Intercept to 0,0 " << circle.checkIntercept(0,0) << endl;
    circle.offsetCentre(-3.0,-3.0);//Lets move centre back 0.0 0.0 so intercepts
    cout << "Intercept to 0,0 " << circle.checkIntercept(0,0) << endl;


//  Inheritance - Ex03
//  ------------------
//  This question can not be solved without the concept of polymorphism
//  We revisit this in Week 04
    std::vector<Shape* > shapes;

    //! We are using new to directly get the pointer to the object
    //! Alternative is to create object first and then obtain the
    //! memory address via "&" address of operator
    shapes.push_back(new Rectangle(3.0,3.0));
    shapes.push_back(new Circle(3.0));

    double x=0,y=0;
    cout << "Enter x and y coordinate of point ie ENTER: 5.0 3.0 " << endl;
    std::cin >> x >> y;


    //!  s->getDescription() and (*s).getDescription() the same

    for (auto s : shapes) {
        cout << s->getDescription();
        if (s->checkIntercept(x,y)){
            cout << " intersects :";
        }
        else {
            cout << " does not intersect:";
        }
        cout << x << "," << y << endl;
    }

    return 1;
}
