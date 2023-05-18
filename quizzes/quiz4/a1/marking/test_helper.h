#ifndef TEST_HELPER_H
#define TEST_HELPER_H

#include "a1_types.h"
#include <cmath>

using namespace pfms::nav_msgs;

/////////////////////////////////////////////////////////////////////////////////////
/// \brief Assembles UGV odometry message, Only for the purposes of this testing do we piggyback on this message and send seq to be 1 for UGV
/// \param x - position x
/// \param y - position y
/// \param yaw - yaw in radians
/// \return assembled odo message (with zero for velocity)
Odometry populateOdoUGV(double x, double y, double yaw){
    Odometry odo;
    odo.seq=1;//Only for the purposes of this testing do we piggyback on this message and send seq to be 0 or 1
    odo.x=x;
    odo.y=y;
    odo.yaw=yaw;
    odo.vx=0;
    odo.vy=0;
    return odo;
}

/////////////////////////////////////////////////////////////////////////////////////
/// \brief Assembles UAV odometry message, Only for the purposes of this testing do we piggyback on this message and send seq to be 2 for UAV
/// \param x - position x
/// \param y - position y
/// \param yaw - yaw in radians
/// \return assembled odo message (with zero for velocity)

Odometry populateOdoUAV(double x, double y, double yaw){
    Odometry odo;
    odo.seq=2;//Only for the purposes of this testing do we piggyback on this message and send seq to be 0 or 1
    odo.x=x;
    odo.y=y;
    odo.yaw=yaw;
    odo.vx=0;
    odo.vy=0;
    return odo;
}

#endif // TEST_HELPER_H
