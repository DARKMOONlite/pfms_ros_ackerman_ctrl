// #include <iostream>
// #include <vector>

// #include "rectangle.h"
// #include "triangle.h"
// #include "circle.h"

// using std::cout;
// using std::endl;
// using std::vector;

// int main () {

//     vector<Shape*> shapes;
//     Triangle triangle(1,1);
//     Shape *trptr = &triangle;
//     Rectangle square(1,1);
//     Shape *sqptr = &square;
//     Circle circle(1);
//     Shape *cirptr = &circle;


//     shapes.push_back(trptr);
//     shapes.push_back(sqptr);
//     shapes.push_back(cirptr);

//     for(int i = 0; i < shapes.size(); i++){
//         std::cout << shapes.at(i)->getArea() << std::endl;
//     }

// }
#include <string.h>
#include <iostream>
#include <vector>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"

int main(){
    int Num_Rectangles, Num_Circles, Num_Triangles;
    int max_length;
    std::vector<Shape*> shapes;

    std::cout << "Number of Rectangles: " << std::endl;
    std::cin >> Num_Rectangles;
    std::cout << "Number of Triangles: " << std::endl;
    std::cin >> Num_Triangles;    
    std::cout << "Number of Circles: " << std::endl;
    std::cin >> Num_Circles;
    std::cout << "Maximum length: " << std::endl;
    std::cin >> max_length;
    std::vector<Rectangle> rectangle;
    std::vector<Triangle> triangle;
    std::vector<Circle> circle;
    rectangle.reserve(Num_Rectangles); //Reserve space so that we can create pointers to the elements in the same loop without worrying about them being dereferenced
    triangle.reserve(Num_Triangles);
    circle.reserve(Num_Circles);
    for(int i = 0; i < Num_Rectangles; i++){
        rectangle.push_back(Rectangle((float)rand()/RAND_MAX*max_length, (float)rand()/RAND_MAX*max_length));
        rectangle.at(i).setCentre((float)rand()/RAND_MAX*max_length/2, (float)rand()/RAND_MAX*max_length/2);
        shapes.push_back(&rectangle.at(i));
    }
    for(int i = 0; i <Num_Triangles; i++){
        triangle.push_back(Triangle((float)rand()/RAND_MAX*max_length, (float)rand()/RAND_MAX*max_length));
        triangle.at(i).setCentre((float)rand()/RAND_MAX*max_length/2, (float)rand()/RAND_MAX*max_length/2);
        shapes.push_back(&triangle.at(i));
    }
    for(int i = 0; i < Num_Circles; i++){
        circle.push_back(Circle((float)rand()/RAND_MAX*max_length));
        circle.at(i).setCentre((float)rand()/RAND_MAX*max_length/2, (float)rand()/RAND_MAX*max_length/2);
        shapes.push_back(&circle.at(i));
    }
 


    for(auto i : shapes){
        std::cout << i->getArea() << std::endl;
    }
    for (auto i : shapes){
        std::cout << i->checkIntercept(1,1) << std::endl;
    }
    
    


}