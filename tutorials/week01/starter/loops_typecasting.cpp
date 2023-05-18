// BONUS Exercise
// Includes std::cout and friends so we can output to console
#include <iostream>
#include <vector>
// Every executable needs a main function which returns an int
int main () {
    char x[] = "41012";
    // Create an string array (char[]) x to value “41012”
    int y;
    int z;
    // Can we typecast to integer each value (and [print] what the value of each integer is?) (int)(x[i])
    for(int i = 0; i < sizeof(x)-1; i++) {
        y = (int)x[i];
        std::cout << y << std::endl;
        z += y;
    }

    // Add up all the elements (as numbers)?
    std::cout << z << std::endl;
    // Can we count number of elements less than 2
    // Use a for loop
    int less_than_two=0;
    for(int i = 0; i < sizeof(x)-1; i++) {
        if(x[i] < 50){less_than_two++;}
    }
    std::cout << less_than_two << std::endl;
    // Use a while loop
    int less_than_two2 = 0;

    int j = 0;
    while(j < sizeof(x)-1){

        if(x[j] < 50){less_than_two2++;}
         
         
         j++;
    }
    std::cout << "less that two 2 = "<<less_than_two2 << std::endl;
    // ADVANCED: Use a for range loop
    std::vector<int> v = {0,1,2,3,4};
    int less_than_two3 = 0;
        std::cout << "for range loop" << std::endl;        
    for(int i :v){
    std::cout << i << std::endl;
    if(x[i]< 50){less_than_two3++;}
}
std::cout << less_than_two3 << std::endl;

int less_than_two4 = 0;


 //! This one keeps it as a string until the if statement is executed
std::string str = x;
    for(char c : str){
        std::cout << c << std::endl;
        if(c < 50){less_than_two4++;}
    }
std::cout << less_than_two4 << std::endl;

    // Note must have the following line to the CMakeList.txt to enable C++11 features
    // set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

    // Main function should return an integer
    return 0;
}
