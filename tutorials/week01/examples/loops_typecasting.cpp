// Includes std::cout and friends so we can output to console
#include <iostream>

// Every executable needs a main function which returns an int
int main () {

    // Create an string array (char[]) x to value “41012”
    char x[] = "41012";

    // Can we typecast to integer each value (and [print] what the value of each integer is?) (int)(x[i])
    for (int i = 0; x[i]!=0; i++) {
        std::cout << "\'" << x[i] << "\' = " << static_cast<int>(x[i]) << std::endl;
    }

    // Add up all the elements (as numbers)?
    int count = 0;
    for (int i = 0; x[i]!=0; i++) {
        count += (x[i] - '0');
    }
    std::cout << "Sum of digits [" << x << "] is " << count << std::endl;

    // Can we count number of elements less than 2
    // Use a for loop
    count = 0;
    for (int i = 0; x[i]!=0; i++) {
        if ((x[i] - '0') < 2) {
            count++;
        }
    }
    std::cout << "Number of digits less than 2 in [" << x << "] is " << count << " (for loop)" << std::endl;

    // Use a while loop
    count = 0;
    int idx = 0;
    while (x[idx]!=0) {
        if ((x[idx] - '0') < 2) {
            count++;
        }
        idx++;
    }
    std::cout << "Number of digits less than 2 in [" << x << "] is " << count << " (while loop)" << std::endl;

    // ADVANCED: Use a for range loop
    // Note must add the following line to the CMakeList to enable C++11 features
    // set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    count = 0;
    for (int num : x) {
        if ( num >= '0' && num <'2' ) {
            count++;
        }
    }
    std::cout << "Number of digits less than 2 in [" << x << "] is " << count << " (for range loop)" << std::endl;

    // Main function should return an integer
    return 0;
}
