#ifndef GRID_PROCESSING_H
#define GRID_PROCESSING_H

#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"

class GridProcessing {

public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    grid - Grid to be searched
   */
  GridProcessing(nav_msgs::OccupancyGrid grid);

  /*! @brief Checks wether the origin and destination can be connected with a line, such that line only goes over free space
   * Code from http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
   *
   *  @param[in]    geometry_msgs::Point origin - point of origin in robot coordinate system [m]
   *  @param[in]    geometry_msgs::Point destination - point of origin in robot coordinate system [m]
   *  @return bool  The points can be connected with a line which only goes over free space
   */
    bool checkConnectivity(geometry_msgs::Point origin, geometry_msgs::Point destination);

    /*! @brief Checks wether the origin and destination can be connected with a line, such that line only goes over free space
     * Code from http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
     *
     *  @param[in]    x0 - x point of origin in robot coordinate system [m]
     *  @param[in]    y0 - y point of origin in robot coordinate system [m]
     *  @param[in]    x1 - x point of origin in robot coordinate system [m]
     *  @param[in]    y1 - y point of origin in robot coordinate system [m]
     *  @return bool  The points can be connected with a line which only goes over free space
     */
    bool checkConnectivity(double x0, double y0, double x1, double y1);

    /*! @brief Accepts a new grid
     *  @param[in]    grid  - Grid to be searched
     */
    void setGrid(nav_msgs::OccupancyGrid grid);

private:

    /*! @brief Checks wether cell at x,y (relative to robot) is free
     *
     *  @param[in]    x - x value of cell to check in robot coordinate system [cells]
     *  @param[in]    y - y value of cell to check in robot coordinate system [cells]
     *  @return bool  The cell is free
     */
    bool testCell(int x, int y);

    /*! @brief Checks wether values are equal to precision 1e-5
     *
     *  @param[in]    x1 - 1st value to check
     *  @param[in]    x2 - 2nd value to check
     *  @return bool  The cell is free
     */
    inline bool isEqual(double x1, double x2)
    {
      const double epsilon = 1e-5; /* some small number such as 1e-5 */;
      return std::abs(x1 - x2) <= epsilon * std::abs(x1);
    }

  nav_msgs::OccupancyGrid grid_; /*!< grid that we will be searching through */
  unsigned int height_; /*!< Heights of grid in  [cells] */
  float resolution_; /*!< Resolution of grid in [m] */
  unsigned int xCentre_; /*!< Centre of grid in cells - x axis [cells]*/
  unsigned int yCentre_; /*!< Centre of grid in cells - y axis  [cells]*/
  unsigned int width_; /*!< Width of grid in  [cells] */
};

#endif // IMAGE_PROCESSING_H
