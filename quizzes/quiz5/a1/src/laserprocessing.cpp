#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>
#include <utility>
#include <limits>
using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{
}



//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countObjectReadings()
{
    unsigned int count=0;

    // std::cout << "Range Min: " << laserScan_.range_min << std::endl;
    // std::cout << "Range Max: " << laserScan_.range_max << std::endl;

    for(int i = 0; i < laserScan_.ranges.size(); i++){
        // ROS_INFO_STREAM("Scan Data" << i << ": " << laserScan_.ranges[i]);
        if(!std::isinf(laserScan_.ranges[i]) && laserScan_.ranges[i]> laserScan_.range_min && laserScan_.ranges[i] < laserScan_.range_max){
            count++;
        }
        // Check each lazer scan, if value is not 0 or infinity then increment count

    }

//   std::cout << "Count: " << count <<std::endl;
//   std::cout << "number of samples taken: " << laserScan_.ranges.size() << std::endl;
  return count;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countSegments()
{
    unsigned int count=0;
    bool flip = 0;
    // std::cout <<"Range Min: "<< laserScan_.range_min << std::endl;
    // std::cout <<"Range Max: "<< laserScan_.range_max << std::endl;
    // std::cout <<"Angle Increment: "<< laserScan_.angle_increment << std::endl;

    for(int i = 0; i < laserScan_.ranges.size(); i++){
        
        // ROS_INFO_STREAM("Scan Data" << i << ": " << laserScan_.ranges[i]);
        if(!std::isinf(laserScan_.ranges[i]) && laserScan_.ranges[i]> laserScan_.range_min && laserScan_.ranges[i] < laserScan_.range_max){
            if(flip != 1){
            count++;
            flip = 1;
            }

        }
        else{
            flip = 0;
            
        }
        // Check each lazer scan, if value is not 0 or infinity then increment count

    }

 

    //Check number
    ROS_INFO_STREAM("SEGMENT COUNT: " << count);

  return count;
}



//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
geometry_msgs::Point LaserProcessing::detectClosestCone(){

    geometry_msgs::Point point;
    std::vector<std::pair<double,double>> cone_ranges;
    double dist=0,angle = 0;

    std::vector<std::pair<double,double>> cone_positions;

    unsigned int count=0;
    bool flip = 0;


    for(int i = 0; i < laserScan_.ranges.size(); i++){
        
        // ROS_INFO_STREAM("Scan Data" << i << ": " << laserScan_.ranges[i]);
        if(!std::isinf(laserScan_.ranges[i]) && laserScan_.ranges[i]> laserScan_.range_min && laserScan_.ranges[i] < laserScan_.range_max){
            if(flip != 1){
            count++;
            flip = 1;
            }
            cone_ranges.push_back(std::make_pair(laserScan_.ranges[i],laserScan_.angle_min+laserScan_.angle_increment*i));


        }
        else{
            
            if(flip==1){
                flip = 0;
                if(cone_ranges.size() > 0){
                    for(auto reading : cone_ranges){
                        dist += reading.first;
                        angle += reading.second;
                    }
                    dist = dist/cone_ranges.size();
                    angle = angle/cone_ranges.size();
                    cone_ranges.clear();
                    cone_positions.push_back(std::make_pair(dist, angle));
                    
                }
            }

        }
        // Check each lazer scan, if value is not 0 or infinity then increment count

    }
    double Min = std::numeric_limits<double>::max();
    int Index = 0;
    for( int i = 0; i <cone_positions.size();i++){
        if(cone_positions.at(i).first< Min){
            Min = cone_positions.at(i).first;
            Index = i;
        }
    }

    point.x = cone_positions.at(Index).first* cos(cone_positions.at(Index).second);
    point.y = cone_positions.at(Index).first* sin(cone_positions.at(Index).second);

    return point;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
geometry_msgs::Point LaserProcessing::detectRoadCentre(){



    geometry_msgs::Point point1 , point2;
    std::vector<std::pair<double,double>> cone_ranges;
    double dist=0,angle = 0;

    std::vector<std::pair<double,double>> cone_positions;

    unsigned int count=0;
    bool flip = 0;


    for(int i = 0; i < laserScan_.ranges.size(); i++){
        
        // ROS_INFO_STREAM("Scan Data" << i << ": " << laserScan_.ranges[i]);
        if(!std::isinf(laserScan_.ranges[i]) && laserScan_.ranges[i]> laserScan_.range_min && laserScan_.ranges[i] < laserScan_.range_max){
            if(flip != 1){
            count++;
            flip = 1;
            }
            cone_ranges.push_back(std::make_pair(laserScan_.ranges[i],laserScan_.angle_min+laserScan_.angle_increment*i));


        }
        else{
            
            if(flip==1){
                flip = 0;
                if(cone_ranges.size() > 0){
                    for(auto reading : cone_ranges){
                        dist += reading.first;
                        angle += reading.second;
                    }
                    dist = dist/cone_ranges.size();
                    angle = angle/cone_ranges.size();
                    cone_ranges.clear();
                    cone_positions.push_back(std::make_pair(dist, angle));
                    
                }
            }

        }
        // Check each lazer scan, if value is not 0 or infinity then increment count

    }
  
    std::pair<double,int> Min = std::make_pair(std::numeric_limits<double>::max(),0);
    std::pair<double,int> Min2= std::make_pair(std::numeric_limits<double>::max(),0);

    for( int i = 0; i <cone_positions.size();i++){
        if(cone_positions.at(i).first< Min2.first){
            Min2.first = cone_positions.at(i).first;
            Min2.second = i;
            if(Min2 < Min){ //ensures we have the 2 closest values
                std::swap(Min2, Min);
            }
        }
    }
    
    point1.x = cone_positions.at(Min.second).first* cos(cone_positions.at(Min.second).second);
    point1.y = cone_positions.at(Min.second).first* sin(cone_positions.at(Min.second).second);
    point2.x = cone_positions.at(Min2.second).first* cos(cone_positions.at(Min2.second).second);
    point2.y = cone_positions.at(Min2.second).first* sin(cone_positions.at(Min2.second).second);
    geometry_msgs::Point point; //{(point1.x+point2.x)/2,(point1.y+point2.y)/2,0};
    point.x = (point1.x+point2.x)/2;
    point.y = (point1.y+point2.y)/2;
    return point;
}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan){
    laserScan_=laserScan;
}


geometry_msgs::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}
