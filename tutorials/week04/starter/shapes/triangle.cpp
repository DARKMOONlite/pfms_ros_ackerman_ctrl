#include "triangle.h"
#include <string.h>
#include <iostream>

Triangle::Triangle(double width, double height):
    width_(width), height_(height)
{
    description_ = "isoc triangle";
}

void Triangle::setHeightWidth(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}

//! Should this work for triangle?
bool Triangle::checkIntercept(double x, double y){
 double A[2] = {centreX_,2*height_/3+centreY_};
 double B[2] = {-width_/2+centreX_, -height_/3+centreY_};
 double C[2]= {width_/2+centreX_, -height_/3+centreY_};
 std::cout<<"Points: A:" << A[0] << " " << A[1] << std::endl;
 std::cout<<"B:" << B[0] << " " << B[1] << std::endl;
 std::cout<<"C:" << C[0] << " "<< C[1] << std::endl;
double B_2[2] = {B[0]-A[0], B[1] - A[1]};
double C_2[2] = {C[0]-A[0], C[1] - A[1]};
double P_2[2] = {x-A[0], y-A[1]};
double scalar = B_2[0]*C_2[1]-B_2[1]*C_2[0];
double Omega[3] ={(P_2[0]*(B_2[1]-C_2[1])+P_2[1]*(C_2[0]-B_2[0])+B_2[0]*C_2[1]-C_2[0]*B_2[1])/scalar,
                 (P_2[0]*C_2[1]-P_2[1]*C_2[0])/scalar,
                 (P_2[1]*B_2[0]-P_2[0]*B_2[1])/scalar}; 
if(Omega[0]>=0 && Omega[0] <=1){
if(Omega[1]>=0 && Omega[1] <=1){
if(Omega[2]>=0 && Omega[2] <=1){
    return(1);
}
}
}
 return(0);
}
