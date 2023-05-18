#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

// Part of the system
#include "a1_types.h"
#include "linkcommand.h"

//Student defined libraries
#include "ackerman.h"

// Some helper header for assembling messages and testing
#include "a1_test_helper.h"
#include "ackerman_test.h"


using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(AckermanExTest, FirstGoal) {

    LinkCommand* linkCommand = new LinkCommand;
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=10,y=0;
    pfms::geometry_msgs::Point pt{10,0};

    controllers.front()->setTolerance(0.5);

    bool reachable = controllers.at(0)->setGoal(pt);
    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_TRUE(reachable);
    ASSERT_NEAR(dist,10.2646,0.5);
    ASSERT_NEAR(t,3.53951,1.0);

    bool reached = controllers.at(0)->reachGoal();

    ASSERT_TRUE(reached);

    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    double distance = pow ( pow(odo.x-pt.x,2) + pow(odo.y-pt.y,2),0.5);

    std::cout << "Distance to goal:" << distance << std::endl;

    ASSERT_NEAR(distance,0,0.5);

}


TEST(AckermanExTest, SecondGoal){


    //   LinkCommand* linkCommand = new LinkCommand;
    // {
    //     // Odometry odo = populateOdoUGV(0,2,0);
    //     //linkCommand->writeCommand(odo);
    // }





    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    pfms::geometry_msgs::Point pt{9,8}; 
    controllers.front()->setTolerance(0.5);
    bool reachable = controllers.at(0)->setGoal(pt);
    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();

    ASSERT_TRUE(reachable);
    ASSERT_NEAR(dist,19.1748,0.5);
    ASSERT_NEAR(t,6.5893,1.0);

    bool reached = controllers.at(0)->reachGoal();

    ASSERT_TRUE(reached);


    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    double distance = pow ( pow(odo.x-pt.x,2) + pow(odo.y-pt.y,2),0.5);

    std::cout << "Distance to goal:" << distance << std::endl;

    ASSERT_NEAR(distance,0,0.5);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
