#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{
}


geometry_msgs::Point LaserProcessing::detectPositionClosest(){

    geometry_msgs::Point point;
    double closestRange =laserScan_.range_max;

    for (unsigned int i = 0; i < laserScan_.ranges.size(); ++i)
    {

        if(laserScan_.ranges.at(i)< closestRange)
          point = polarToCart(i);
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

double LaserProcessing::angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}
