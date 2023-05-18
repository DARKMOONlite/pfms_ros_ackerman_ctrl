// Includes std::cout and friends so we can output to console
#include <iostream>

// Every executable needs a main function which returns an int
int main () {

    // Create an string array (char[]) x to value “41012”
    char x[] = "41012";

    // Switch on items 4,1,0,2 and print word for value (what type needs to be in case clause?)
    // ...Argument to switch statement must be integer (chars can be treated as intergers)
    for (int i = 0; x[i]!=0; i++) {
        switch (x[i]) {
        case '4':
            std::cout << "four ";
            break;
        case '1':
            std::cout << "one ";
            break;
        case '0':
            std::cout << "zero ";
            break;
        case '2':
            std::cout << "two ";
            break;
        default:
            std::cout << "other ";
            break;
        }
    }
    std::cout << std::endl;

    // Main function should return an integer
    return 0;
}
