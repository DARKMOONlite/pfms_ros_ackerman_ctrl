#include <iostream>

int main(){

struct Movie{
    int rating;
    int year;
};
Movie myMovie{2,2021};

std::cout << myMovie.rating << std::endl;
std::cout << myMovie.year << std::endl;




class Rectangle{
public: //can be accessed outside of class
    void setValues (int, int);
    int getArea(void);
    int getPerimeter(void);
private: //is used within the class. It's good to put most functions in private, and allow changes of that data using functions.
    int width, height;

};






return 0;
}