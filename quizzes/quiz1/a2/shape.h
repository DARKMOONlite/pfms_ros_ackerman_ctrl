#ifndef SHAPE_H
#define SHAPE_H

#include <string>

/*!
 *  \ingroup   ac_shapre Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    Shape();
    /**
     * @brief Function that sets centre of shape
     * @param x in [m]
     * @param y in [m]
     */
    virtual void setCentre(double x, double y);
    /**
     * @brief Returns description of shape
     * @return description
     */
    std::string getDescription();
   // make getArea() a method that
    // 1) Base class Shape defines
    // 2) Needs to be in Rectangle
    // 3) makes Shape a pure virtual class
    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    virtual double getArea (void);

//    1) TASK
//    Correct the missing access specifiers of base class [shape](./b2/shape.h) so that the immplementaion of the
//     [`Rectangle` constructor](./a2/rectangle.cpp) can still access the required member varaible of
//     `Shape` but have it restricted to users of a `Rectangle` object.
protected:
    std::string description_;//!< description of shape
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
