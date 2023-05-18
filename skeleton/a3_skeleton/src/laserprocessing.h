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


  /*! TASK 3
   * @brief Return position of closest reading
   *
   * The position should be the location of the closest reading
   * @return point - point at location of closest reading
   */
  geometry_msgs::Point detectPositionClosest();


  /*! @brief Accepts a new laserScan
   *  @param[in]    laserScan  - laserScan to be processed
   */
  void newScan(sensor_msgs::LaserScan laserScan);



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

private:
    sensor_msgs::LaserScan laserScan_;

};

#endif // DETECTCABINET_H
