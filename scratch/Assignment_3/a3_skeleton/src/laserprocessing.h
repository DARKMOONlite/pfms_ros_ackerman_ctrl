#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "tf/transform_datatypes.h"
class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::LaserScan laserScan);

/**
 * @brief Detects the closest cone within provided arc. Default 360
 * 
 * @param arc Allows you to pass in an arc region to scan, centred on the front of the vehicle
 * @return geometry_msgs::Point 
 */
  geometry_msgs::Point detectPositionClosest(int arc=180);



/**
 * @brief Deprecated, functionality replaced with the ability to pass angle to detectPositionClosest
 * 
 * @return geometry_msgs::Point 
 */
geometry_msgs::Point detectPositionClosestSonar();




  /*! @brief Accepts a new laserScan
   *  @param[in]    laserScan  - laserScan to be processed
   */
  void newScan(sensor_msgs::LaserScan laserScan);
/**
 * @brief Returns Average Point of all Cones Detected
 * 
 * @return std::vector<geometry_msgs::Point> 
 */
std::vector<geometry_msgs::Point> detectPoints(int arc=180);

/**
 * @brief Returns all detected points from the latest sensor_msgs/LaserScan msg
 * 
 * @return std::vector<geometry_msgs::Point> 
 */
std::vector<geometry_msgs::Point> detectAllPoints();

private:
  /*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
   geometry_msgs::Point polarToCart(unsigned int index);

   /*! @brief Given two points (only x,y are used), returns the slope slope of the lines connecting them
    *  @param[in] p1 - first point
    *  @param[in] p2 - second point
    *  @return slope of line between the points in radians
    */
  double angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);
/**
 * @brief Checks if value is within minimum and maximum range and not inf
 * 
 * @param index which value to check
 * @return true within range
 * @return false outside of range
 */
bool withinRanges(int index);
/**
 * @brief Determines if index provided is within the speficied arc
 * 
 * @param index index of laser reading
 * @param arc size of arc to focus on
 * @return true 
 * @return false 
 */
bool withinArc(int index,int arc);

private:
/**
 * @brief Storage for Laser Data
 * 
 */
    sensor_msgs::LaserScan laserScan_;

};

#endif // DETECTCABINET_H
