#include "radialdetectionogmap.h"
#include <iostream>

RadialDetectionOgMap::RadialDetectionOgMap(unsigned int h, unsigned int w, std::vector<signed char> data):
  width_(w),xCentre_(h/2),yCentre_(w/2),data_(data)
{

}


bool RadialDetectionOgMap::testCell(int x, int y, int& index){

  unsigned int col =x + xCentre_;
  unsigned int row =y + yCentre_;
  index = row * width_ + col;
//  std::cout << static_cast<int>(data_.at(index))  << "," ;
  return (data_.at(index) ==100) ? true : false; //Return true if value 100 and false otherwise;
}

// Implementing Mid-Point Circle Algorithm
// C++ implementation ported from https://www.geeksforgeeks.org/mid-point-circle-drawing-algorithm/
int RadialDetectionOgMap::midPointCircleDetectOccupied(int rmax)
{

//  /// analysis
//  {
//    int x = -static_cast<int>(width_/2);
//    int y = -static_cast<int>(width_/2);
//    int index =0;
//    std::cout << "[x,y,val,index]=[" << x << "," << y << ",";
//    bool OK= testCell(x,y,index);
//    std::cout << index << std::endl;
//    if(OK){
//      std::cout << "Free" << std::endl;
//    }
//  }

//  {
//    int x = -static_cast<int>(width_/2);
//    int y = static_cast<int>(width_/2)-1;
//    int index =0;
//    std::cout << "[x,y,index]=[" << x << "," << y << ",";
//    bool OK= testCell(x,y,index);
//    std::cout << index << std::endl;
//    if(OK){
//      std::cout << "Free" << std::endl;
//    }
//  }

//  {
//    int x = static_cast<int>(width_/2)-1;
//    int y = static_cast<int>(width_/2)-1;
//    int index =0;
//    std::cout << "[x,y,index]=[" << x << "," << y << ",";
//    bool OK= testCell(x,y,index);
//    std::cout << index << std::endl;
//    if(OK){
//      std::cout << "Free" << std::endl;
//    }
//  }

//  {
//    int x = static_cast<int>(width_/2)-1;
//    int y = -static_cast<int>(width_/2);
//    int index =0;
//    std::cout << "[x,y,index]=[" << x << "," << y << ",";
//    bool OK= testCell(x,y,index);
//    std::cout << index << std::endl;
//    if(OK){
//      std::cout << "Free" << std::endl;
//    }
//  }

//  return -1;

  for(int r=0;r<=rmax;r++){
      int x = r, y = 0;

      int index =0;

      if (testCell(x,y,index)){
        return index;
      }

      // When radius is zero only a single
      // point will be examined
      if (r > 0)
      {
          if (testCell(x,-y,index)){
            return index;
          }

          if (testCell(x,y,index)){
            return index;
          }

          if (testCell(-x,x,index)){
            return index;
          }

      }

      // Initialising the value of P
      int P = 1 - r;
      while (x > y)
      {
          y++;

          // Mid-point is inside or on the perimeter
          if (P <= 0)
              P = P + 2*y + 1;
          // Mid-point is outside the perimeter
          else
          {
              x--;
              P = P + 2*y - 2*x + 1;
          }

          // All the perimeter points have already been printed
          if (x < y)
              break;

          if (testCell(x,y,index)){
            return index;
          }
          if (testCell(-x,y,index)){
            return index;
          }
          if (testCell(x,-y,index)){
            return index;
          }
          if (testCell(-x,-y,index)){
            return index;
          }

          // If the generated point is on the line x = y then
          // the perimeter points have already been visited
          if (x != y)
          {
            if (testCell(y,x,index)){
              return index;
            }
            if (testCell(-y,x,index)){
              return index;
            }
            if (testCell(y,-x,index)){
              return index;
            }
            if (testCell(-y,-x,index)){
              return index;
            }


          }
      }
  }

  return -1;

}
