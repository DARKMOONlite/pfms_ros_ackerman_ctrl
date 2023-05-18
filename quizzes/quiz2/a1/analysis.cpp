#include "analysis.h"

using std::vector;
using std::pair;
using std::make_pair;
using std::sort;
using std::find;
using std::any_of;

//! @todo
//! TASK 1 - We need to store cars in the Analysis class, how to do this?
Analysis::Analysis(std::vector<CarInterface*> cars) :
    cars_(cars),raceDisplay_(nullptr)
{

}

Analysis::Analysis(std::vector<CarInterface*> cars,std::shared_ptr<DisplayRace> raceDisplay) :
    cars_(cars),raceDisplay_(raceDisplay)
{

}

//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::sortByOdometry(){

    vector<unsigned int> order;//Creating a vector, same size as cars with all zeros
    vector<pair<unsigned int, unsigned int>> Index; 
    for(int i = 0; i < cars_.size(); i++){
        Index.push_back(make_pair(cars_.at(i)->getOdometry(),i));
        
    }
    //Sort the pairs 
    sort(Index.rbegin(), Index.rend());

    for(auto pair : Index){
        order.push_back(pair.second);
    }
 


    return order;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::dragRace(double distance){
   unsigned int Place = 0 ;
    vector<unsigned int> Odo;
    vector<bool> Done(cars_.size(),false);
    vector<unsigned int> order (cars_.size(),0);
    vector<pair<unsigned int, unsigned int>> Index; 
    
    for(auto car : cars_){ // for each car we are looking at
        Odo.push_back(car->getOdometry());
    }
   
    while(find(Done.begin(),Done.end(),false)!= Done.end()){ //While Done contains any vehicle not done
        for(int i = 0; i < cars_.size(); i++){ 
           
            if(Done.at(i)==false){
                   
            if(cars_.at(i)->getOdometry() <= Odo.at(i)+1000){ //makes care accelerate
                cars_.at(i)->accelerate();
                
            }
            else{ //When it passes it. Set Done to true
                
                Done.at(i) = true;
                order.at(i) = Place;
                Place++;
               
                
                }
            }
            //else{cars_.at(i)->decelerate();}
        }

        
    }
    
    
   
    return order;
}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars(){
    vector<bool> Done(cars_.size(),false);
    while(find(Done.begin(),Done.end(),false)!= Done.end()){ //
        for(int i = 0; i < cars_.size(); i++){ // for each car we are looking at
            if(cars_.at(i)->getCurrentSpeed() > 0){
                cars_.at(i)->decelerate();
            }
            else{Done.at(i)=true;}
        }
    }
    
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description

std::vector<unsigned int> Analysis::zeroTopZeroRace(){
    int Place = 0;
    vector<int> Done(cars_.size(),false);
    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros
    while(any_of(Done.begin(),Done.end(),[](unsigned int y){return (y==1) || (y==0);})){ //! heereeeee
    //while(find(Done.begin(),Done.end(),false)!= Done.end()){
        
        for(int i = 0; i < cars_.size(); i++){
           

            if(Done.at(i)==0){
                
                cars_.at(i)->accelerate();
                if(cars_.at(i)->getCurrentSpeed() >= cars_.at(i)->getTopSpeed()){
                    Done.at(i)=1;
                }

            }
            if(Done.at(i)==1){
                cars_.at(i)->decelerate();
                if(cars_.at(i)->getCurrentSpeed() <=1){
                    Done.at(i) = 2;
                    order.at(i) = Place;
                    Place++;

                }

            }




        }



    //}

    }





    return order;
}

// Demo code
void Analysis::demoRace(){


    //This is an example of how to draw 3 cars moving
    // accelerate 300 times
    unsigned int count=0;

    while(count < 300){
        for(auto car : cars_){
          car->accelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

    // decelerate 600 times
    count =0;
    while(count < 600){
        for(auto car : cars_){
          car->decelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

}
