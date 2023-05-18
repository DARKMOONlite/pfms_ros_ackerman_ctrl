#ifndef RADIALDETECTIONOGMAP_H
#define RADIALDETECTIONOGMAP_H

#include <vector>

class RadialDetectionOgMap
{
public:
  RadialDetectionOgMap(unsigned int h, unsigned int w, std::vector<signed char> data);

  /*! @brief efficient search for cells on a radius and detection of occupied cell
   *
   * Midpoint cicrle algorithm performed to rmax
   */
  int midPointCircleDetectOccupied(int rmax);

private:

  bool testCell(int x, int y, int& index);

  unsigned int xCentre_;
  unsigned int yCentre_;
  unsigned int width_;

  std::vector<signed char> data_;

  // Implementing Mid-Point Circle Algorithm
  // C++ implementation ported from https://www.geeksforgeeks.org/mid-point-circle-drawing-algorithm/

};

#endif // RADIALDETECTIONOGMAP_H
