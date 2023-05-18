#include <limits> // for infinity
#include "grid_processing.h"
#include <iostream>

GridProcessing::GridProcessing(nav_msgs::OccupancyGrid grid):
  grid_(grid),
  height_(grid.info.height),
  resolution_(grid.info.resolution),
  xCentre_(grid.info.width/2),
  yCentre_(grid.info.height/2),
  width_(grid.info.width){

}

bool GridProcessing::checkConnectivity(double x0, double y0, double x1, double y1)
{
  x0/=static_cast<double>(resolution_);
  y0/=static_cast<double>(resolution_);
  x1/=static_cast<double>(resolution_);
  y1/=static_cast<double>(resolution_);

  x0+=xCentre_;
  y0+=yCentre_;
  x1+=xCentre_;
  y1+=yCentre_;

//  std::cout << "[x0,y0,x1,y1][w,h]=[" << x0 << "," << y0 << "," <<
//               x1 << "," << y1 << "] [" <<
//               width_ << "," << height_ << "]" << std::endl;

   //Let's check the points to be searched are not outside the gridmap
  if((x0<0)||(x0>width_)||(y0<0)||(y0>height_)){
    return false;
  }
  if((x1<0)||(x1>width_)||(y1<0)||(y1>height_)){
    return false;
  }

  double dx = fabs(x1 - x0);
  double dy = fabs(y1 - y0);

  int x = int(floor(x0));
  int y = int(floor(y0));

  int n = 1;
  int x_inc, y_inc;
  double error;

  if (isEqual(dx,0))
  {
      x_inc = 0;
      error = std::numeric_limits<double>::infinity();
  }
  else if (x1 > x0)
  {
      x_inc = 1;
      n += int(floor(x1)) - x;
      error = (floor(x0) + 1 - x0) * dy;
  }
  else
  {
      x_inc = -1;
      n += x - int(floor(x1));
      error = (x0 - floor(x0)) * dy;
  }

  if (isEqual(dy,0))
  {
      y_inc = 0;
      error -= std::numeric_limits<double>::infinity();
  }
  else if (y1 > y0)
  {
      y_inc = 1;
      n += int(floor(y1)) - y;
      error -= (floor(y0) + 1 - y0) * dx;
  }
  else
  {
      y_inc = -1;
      n += y - int(floor(y1));
      error -= (y0 - floor(y0)) * dx;
  }

  for (; n > 0; --n)
  {
       if(!testCell(x,y)){
         return false;//Let's break immediately here, search no longer required
       }

      if (error > 0)
      {
          y += y_inc;
          error -= dx;
      }
      else
      {
          x += x_inc;
          error += dy;
      }
  }

  return true;
}

bool GridProcessing::testCell(int x, int y){

  unsigned int index =  y * width_ + x; //Is a row major
//  std::cout << "[x,y,index,val]=[" << x << "," << y << "," <<
//               index << "," <<
//               static_cast<int>(grid_.data.at(index)) << "]" << std::endl;
  if(grid_.data.at(index)<0){
    return false;
  }
  return (grid_.data.at(index)<10)?true:false;
}


bool GridProcessing::checkConnectivity(geometry_msgs::Point origin, geometry_msgs::Point destination)
{
 return checkConnectivity(origin.x, origin.y, destination.x, destination.y);
}

void GridProcessing::setGrid(nav_msgs::OccupancyGrid grid){
  grid_ = grid;
  xCentre_=grid.info.width/2;
  yCentre_=grid.info.height/2;
  width_=grid.info.width;
  height_=grid.info.height;
  resolution_=grid.info.resolution;
  return;
}
