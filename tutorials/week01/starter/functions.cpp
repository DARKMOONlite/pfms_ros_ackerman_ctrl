// Includes std::cout and friends so we can output to console
#include <iostream>

// Ex01. 
//* Create a function that accepts a double value as a parameter and
//* Returns a bool value if the double is greater than zero and the square value instead of initial passed value.

// Ex02
//* Create an additional function that accepts a double value as a parameter and
//* Returns bool value if the double is greater than zero, the square value of the initially passed value, and the passed value incremented by one
double Double_Check(double &input, double &square){
    bool check = input > 0;
    square = input*input;
    input++;
    return(check);
    


}

// Every executable needs a main function which returns an int
int main () {

    double test = 3;
    double test2;

    bool x = Double_Check(test, test2);

    std::cout << test << std::endl;
    std::cout << x << std::endl;
    std::cout << test2 << std::endl;  


	//Call the functions, obtain values and print to screen (using std::cout) 


    return 0;
}




