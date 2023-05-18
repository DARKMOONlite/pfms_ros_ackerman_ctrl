#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "quadcopter.h"
#include "ackerman.h"
#include "a1_types.h"
#include <cmath>

//using namespace std;
using namespace pfms::nav_msgs;


TEST(ConstructorTest, Quadcopter) {

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    ASSERT_EQ(controllers.front()->getPlatformType(),pfms::PlatformType::QUADCOPTER);
    ASSERT_FLOAT_EQ(controllers.front()->distanceTravelled(),0.0f);
    ASSERT_FLOAT_EQ(controllers.front()->timeInMotion(),0.0f);
}

TEST(ConstructorTest, Ackerman) {

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    ASSERT_EQ(controllers.front()->getPlatformType(),pfms::PlatformType::ACKERMAN);
    ASSERT_FLOAT_EQ(controllers.front()->distanceTravelled(),0.0f);
    ASSERT_FLOAT_EQ(controllers.front()->timeInMotion(),0.0f);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
