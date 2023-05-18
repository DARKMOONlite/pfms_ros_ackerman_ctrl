#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{
}


geometry_msgs::Point LaserProcessing::detectPositionClosest(int arc){

    geometry_msgs::Point point;
    double closestRange =laserScan_.range_max;
  

    

    

    for (unsigned int i = 0; i < laserScan_.ranges.size(); ++i)
    {

        if(laserScan_.ranges.at(i)< closestRange && withinArc(i,arc)){
            point = polarToCart(i);
            closestRange = laserScan_.ranges.at(i);
        }
         
    }
    return point;
}

geometry_msgs::Point LaserProcessing::detectPositionClosestSonar(){
 geometry_msgs::Point point;
    double closestRange =laserScan_.range_max;

    for (unsigned int i = 0; i < laserScan_.ranges.size(); ++i)
    {
        
        if(laserScan_.ranges.at(i)< closestRange && (i>laserScan_.ranges.size()/3 && i<laserScan_.ranges.size()*2/3)){
            point = polarToCart(i);
            closestRange = laserScan_.ranges.at(i);
        }
         
    }
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
std::vector<geometry_msgs::Point> LaserProcessing::detectPoints(int arc){
    geometry_msgs::Point point;


    double closestRange =laserScan_.range_max;
    bool flip = 0;
    std::vector<std::vector<geometry_msgs::Point>> Sections;
    
    std::vector<geometry_msgs::Point> newCone;
    // std::cout <<" max & min" << laserScan_.angle_max <<" " << laserScan_.angle_min << std::endl;
    // std::cout <<  laserScan_.angle_increment << " " << laserScan_.ranges.size() << std::endl;
    for(int i = 1; i<laserScan_.ranges.size();i++){
    
        if(withinRanges(i)&&withinArc(i,arc)){
            // if the value is close to the old one 
            if(abs(laserScan_.ranges.at(i) - laserScan_.ranges.at(i-1))<0.5 || !withinRanges(i-1)){
                newCone.push_back(polarToCart(i));
            }
            else{ // if the value is far away from the old one assume new cone
            
                Sections.push_back(newCone);
                newCone.clear();
                newCone.push_back(polarToCart(i));
            }
        }
        else{
            if(newCone.size()>0){ //Pushback the Cone and clear it 
                Sections.push_back(newCone);
                newCone.clear();
            }

        }
    }
    // for(int i = 0; i<laserScan_.ranges.size();i++){
    //     std::cout << laserScan_.ranges.at(i)<< " ";
    // }
    // std::cout << " number of Cones Detected: " << Sections.size() << std::endl;
    std::vector<geometry_msgs::Point> Points;
    for(int i=0; i < Sections.size(); i++){
        double x = 0,y=0;
        for(int j=0; j < Sections.at(i).size(); j++){
            x+=Sections.at(i).at(j).x;
            y+=Sections.at(i).at(j).y;
        }
        x = x/Sections.at(i).size();
        y = y/Sections.at(i).size();
        geometry_msgs::Point Point;
        Point.x = x;
        Point.y = y;
        Point.z = 0;
        Points.push_back(Point);
    }
    // for(int i = 0; i < Points.size(); i++){
    //     std::cout << "Point: " << Points[i].x << " " << Points[i].y << std::endl;
    // }

    return(Points);
}

// && (abs(laserScan_.ranges[i] - laserScan_.ranges[i-1])<0.5)
double LaserProcessing::angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

std::vector<geometry_msgs::Point> LaserProcessing::detectAllPoints(){
    //ROS_DEBUG_STREAM("Running DetectAllPoints");
std::vector<geometry_msgs::Point> points;


for(unsigned int i = 0; i < laserScan_.ranges.size(); ++i){
    if(withinRanges(i)){
        points.push_back(polarToCart(i));
    }
}

return(points);
}

bool LaserProcessing::withinRanges(int index){
    if(!std::isinf(laserScan_.ranges.at(index))&&laserScan_.ranges.at(index)> laserScan_.range_min && laserScan_.ranges.at(index) < laserScan_.range_max ){
        return(true);
    }
    else{return(false);}

}

bool LaserProcessing::withinArc(int index,int arc){
    double minus = (180-arc)/2;
    
    double fraction = minus/180;
    
if(index>(laserScan_.ranges.size()*fraction) && index<laserScan_.ranges.size()*(1-fraction)){
    return(true);
}
else{return(false);}
}


// std::vector<geometry_msgs::Point> LaserProcessing::detectPoints(int arc){
//     geometry_msgs::Point point;


//     double closestRange =laserScan_.range_max;
//     bool flip = 0;
//     std::vector<std::vector<geometry_msgs::Point>> Sections;
    
//     std::vector<geometry_msgs::Point> newCone;
//     //std::cout <<" max & min" << laserScan_.range_max <<" " << laserScan_.range_min << std::endl;
    
//     for(int i = 1; i<laserScan_.ranges.size();i++){
    
//         if(withinRanges(i)){
//             // if the value is close to the old one
//             if(abs(laserScan_.ranges[i] - laserScan_.ranges[i-1])<0.5 || !withinRanges(i-1)){
//                 newCone.push_back(polarToCart(i));
//             }
//             else{ // if the value is far away from the old one assume new cone
//                 Sections.push_back(newCone);
//                 newCone.clear();
//                 newCone.push_back(polarToCart(i));
//             }
//         }
//         else{
//             if(newCone.size()>0){ //Pushback the Cone and clear it 
//                 Sections.push_back(newCone);
//                 newCone.clear();
//             }

//         }
//     }
//     // for(int i = 0; i<laserScan_.ranges.size();i++){
//     //     std::cout << laserScan_.ranges.at(i)<< " ";
//     // }
//     // std::cout << " number of Cones Detected: " << Sections.size() << std::endl;
//     std::vector<geometry_msgs::Point> Points;
//     for(int i=0; i < Sections.size(); i++){
//         double x = 0,y=0;
//         for(int j=0; j < Sections.at(i).size(); j++){
//             x+=Sections.at(i).at(j).x;
//             y+=Sections.at(i).at(j).y;
//         }
//         x = x/Sections.at(i).size();
//         y = y/Sections.at(i).size();
//         geometry_msgs::Point Point;
//         Point.x = x;
//         Point.y = y;
//         Point.z = 0;
//         Points.push_back(Point);
//     }
//     // for(int i = 0; i < Points.size(); i++){
//     //     std::cout << "Point: " << Points[i].x << " " << Points[i].y << std::endl;
//     // }

//     return(Points);
// }
