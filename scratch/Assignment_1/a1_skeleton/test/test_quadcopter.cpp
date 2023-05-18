#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "quadcopter.h"
#include "a1_types.h"
#include <cmath>

#include "a1_test_helper.h"
#include "quadcopter_test.h" //Has the QuadcopterTest setup
#include <iostream>
//using namespace std;
using namespace pfms::nav_msgs;


TEST_F(QuadcopterTest, Simple) {

    //We send the fake odometry, just so we can build tests cases
    Odometry odo = populateOdoUAV(0,0,0);//x=0, y=2, yaw=0;

   pipesFakeOdo_->writeCommand(odo);;// We send fake data

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    
    controllers.push_back(new Quadcopter());

    //Goal location
    pfms::geometry_msgs::Point pt{10,0};

    //Set the goal for controller zero (the Quad)
    bool reachable = controllers.at(0)->setGoal(pt);


    // Get distance to goal
    double dist = controllers.at(0)->distanceToGoal();
    // Get time to goal
    double t = controllers.at(0)->timeToGoal();


    std::cout << "Quadcopter: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;


    ASSERT_TRUE(reachable);// The goal should be reachable
    ASSERT_NEAR(dist,10,0.2);//Distance 10.2646 [m], tolerance 0.2 [m] for the answer
    ASSERT_NEAR(t,25,0.5); //Time 3.535 [s], tolerance 0.5 [s] for the answer
}


TEST_F(QuadcopterTest, Simple2) {

    Odometry odo = populateOdoUAV(0,0,0);//x=0, y=2, yaw=0;
    pipesFakeOdo_->writeCommand(odo);;// We send fake data

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    pfms::geometry_msgs::Point pt{0,-6.5};

    bool reachable = controllers.at(0)->setGoal(pt);
    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Quadcopter: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_TRUE(reachable);
    ASSERT_NEAR(dist,6.5,0.2);
    ASSERT_NEAR(t,16.25,0.5);
}

TEST_F(QuadcopterTest, ForwardFacing) {

    Odometry odo = populateOdoUAV(0.225,-5.306,127.825*M_PI/180);//x=0.225, y=-5.306, yaw=127deg
    pipesFakeOdo_->writeCommand(odo);;// We send fake data

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    //Goal at x=1,y=4;//This shoudl not be possible, exceeds max driving circle
    pfms::geometry_msgs::Point pt{1,4};


    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Quadcopter: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_NEAR(dist,9.33822,0.2);
    ASSERT_NEAR(t,23.3455,0.5);
}


TEST_F(QuadcopterTest, BackwardFacing) {

    Odometry odo = populateOdoUAV(-5.5,-5.5,-127.825*M_PI/180);//x=0.225, y=-5.306, yaw=127deg
    pipesFakeOdo_->writeCommand(odo);// We send fake data

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    //Goal at x=1,y=4;//This shoudl not be possible, exceeds max driving circle
    pfms::geometry_msgs::Point pt{4.2,4.8};

    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Quadcopter: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_NEAR(dist,14.1485,0.2);
    ASSERT_NEAR(t,35.3712,0.5);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
