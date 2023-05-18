#ifndef LOGGER_H
#define LOGGER_H

#include "a1_types.h"
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

namespace logger {

  /*! @brief Read 2d points (vector of point) from file
   *

   *  @param [in] fileName - file name containing the points logged
   *  @param [in|out] points - Vector of pfms::geometry_msgs::Point, will erase vector prior to reading
   *  @return bool - Indicates if operation was successful
      */
    bool loadPoints(std::string fileName, std::vector<pfms::geometry_msgs::Point>& points);

  /*! @brief Save 2d points (vector of point) to file
   *

   *  @param [in] fileName - file name for points to  be logged
   *  @param [in] points - Vector of pfms::geometry_msgs::Point
   *  @return bool - indicates if operation was successful
      */
      bool savePoints(std::string fileName, std::vector<pfms::geometry_msgs::Point> points);



  /*! @brief Read 3d points (vector of point) from file
   *

   *  @param [in] fileName - file name containing the points logged
   *  @param [in|out] points - Vector of pfms::geometry_msgs::Point3d, will erase vector prior to reading
   *  @return bool - Indicates if operation was successful
      */
    bool loadPoints(std::string fileName, std::vector<pfms::geometry_msgs::Point3d>& points);

  /*! @brief Save 3d points (vector of point) to file
   *

   *  @param [in] fileName - file name for points to  be logged
   *  @param [in] points - Vector of pfms::geometry_msgs::Point3d
   *  @return bool - indicates if operation was successful
      */
      bool savePoints(std::string fileName, std::vector<pfms::geometry_msgs::Point3d> points);


};

#endif // LOGGER_H
