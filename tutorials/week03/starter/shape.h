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
    virtual std::string getDescription();

//    Correct the missing access specifiers of base class [shape]so that the implementaion of the
//     [`Rectangle` constructor]can still access the required member varaible of
//     `Shape` but have it restricted to users of a `Rectangle` object.
protected:
   std::string description_;//!< description of shape
protected:
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
