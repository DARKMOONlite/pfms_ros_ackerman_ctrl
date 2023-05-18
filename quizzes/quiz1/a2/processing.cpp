#include "processing.h"



void removeLargerThanArea(std::vector<Shape*> &shapes, double limit){
    for(int i=0; i<shapes.size(); i++){
        if(shapes.at(i)->getArea() > limit){
            shapes.erase(shapes.begin()+i); //this is a weird workaround because erase requires an iterator.
            i--;          //.at doesn't give itterator but .begin does

        }
    }

}