#include "dataprocessing.h"

DataProcessing::DataProcessing()
{

}

void DataProcessing::findClosestReading(){
    while(true){
        double distance = -1;
        if(radars_.size() == 0){
            std::cout << "No Radars Set" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else{
            for( unsigned int i = 0; i < radars_.size();i++){
                double maxVal = radars_.at(i)->getMaxDistance();
                if(maxVal > distance){
                    distance = maxVal;
                }
            }
            for( unsigned int i = 0; i < radars_.size();i++){
                std::vector<double> data = radars_.at(i)->getData();
                for(auto elem: data){
                    if(elem < distance){
                        distance = elem;
                    }
                }
            }
            std::cout << "Closest Reading: " << distance << std::endl;
        }
    }
}
void DataProcessing::setRadars(std::vector<Radar*> radars){
    radars_ = radars;

}