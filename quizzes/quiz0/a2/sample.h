#include <iostream> // Includes std::cout and friends so we can output to console
#include <cmath>   // Includes the math library

class Sample {
public: 
// 1) TASK: Add the special member function (the Constructor) to the `Sample` class
//  the constructor needs to take a double value as a parameter and assign this value to the member variable `value_`.
    Sample(double value);
    void setValue (double value); //! Assigns value to the private member value_
    double readValue (void); //! returns the value of private member value_
private:
    double value_;
};
