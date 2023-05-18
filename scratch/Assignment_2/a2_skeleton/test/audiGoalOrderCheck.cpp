#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "a1_types.h"

//Student defined libraries
#include "ackerman.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "linkcommand.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(AckermanCheckGoalOrder, Set1) {
 LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

std::vector<pfms::geometry_msgs::Point> goals;
goals.push_back({10, -2});
goals.push_back({10, 6});
goals.push_back({0 ,-2});
goals.push_back({-3, -20});
goals.push_back({-10, -20});
goals.push_back({-10, 40});
goals.push_back({-17, 40});
goals.push_back({-20, 9});
goals.push_back({-15, 5});
goals.push_back({0, 2});

Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::ADVANCED);
    mission.setGoals(goals,pfms::PlatformType::ACKERMAN);

    std::vector<Point> ordered_goals;

    ordered_goals = mission.FindBestPath(0);

    for(int i = 0; i < ordered_goals.size();i++){
        std::cout << ordered_goals[i].x << " " << ordered_goals[i].y << std::endl;
    }

    ASSERT_EQ(ordered_goals.size(),goals.size());

    ASSERT_EQ(ordered_goals.at(0).x,goals.at(0).x);
   ASSERT_EQ(ordered_goals.at(1).x,goals.at(1).x);
ASSERT_EQ(ordered_goals.at(7).x,goals.at(2).x);
ASSERT_EQ(ordered_goals.at(9).x,goals.at(3).x);
ASSERT_EQ(ordered_goals.at(8).x,goals.at(4).x);
ASSERT_EQ(ordered_goals.at(6).x,goals.at(5).x);
ASSERT_EQ(ordered_goals.at(5).x,goals.at(6).x);
ASSERT_EQ(ordered_goals.at(4).x,goals.at(7).x);
ASSERT_EQ(ordered_goals.at(3).x,goals.at(8).x);
ASSERT_EQ(ordered_goals.at(2).x,goals.at(9).x);

}


TEST(AckermanCheckGoalOrder, Set2) {
 LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

std::vector<pfms::geometry_msgs::Point> goals;
goals.push_back({-15, 5});
goals.push_back({10, -2});
goals.push_back({10, 6});
goals.push_back({-20, 9});
goals.push_back({0 ,-2});
goals.push_back({-17, 40});
goals.push_back({-10, -20});
goals.push_back({-10, 40});
goals.push_back({0, 2});
goals.push_back({-3, -20});

Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::ADVANCED);
    mission.setGoals(goals,pfms::PlatformType::ACKERMAN);

    std::vector<Point> ordered_goals;

    ordered_goals = mission.FindBestPath(0);

    for(int i = 0; i < ordered_goals.size();i++){
        std::cout << ordered_goals[i].x << " " << ordered_goals[i].y << std::endl;
    }

    ASSERT_EQ(ordered_goals.size(),goals.size());

    ASSERT_EQ(ordered_goals.at(3).x,goals.at(0).x);
   ASSERT_EQ(ordered_goals.at(0).x,goals.at(1).x);
ASSERT_EQ(ordered_goals.at(1).x,goals.at(2).x);
ASSERT_EQ(ordered_goals.at(4).x,goals.at(3).x);
ASSERT_EQ(ordered_goals.at(7).x,goals.at(4).x);
ASSERT_EQ(ordered_goals.at(5).x,goals.at(5).x);
ASSERT_EQ(ordered_goals.at(8).x,goals.at(6).x);
ASSERT_EQ(ordered_goals.at(6).x,goals.at(7).x);
ASSERT_EQ(ordered_goals.at(2).x,goals.at(8).x);
ASSERT_EQ(ordered_goals.at(9).x,goals.at(9).x);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
