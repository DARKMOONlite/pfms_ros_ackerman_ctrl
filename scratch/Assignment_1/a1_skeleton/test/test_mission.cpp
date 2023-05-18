#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "quadcopter.h"
#include "mission.h"

#include "a1_types.h"
#include <cmath>
// Some helper header for assembling messages and testing
#include "a1_test_helper.h"
#include "mission_test.h"

using namespace std;
using namespace pfms::nav_msgs;



///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST_F(MissionTest, Simple) {

    {

        Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
        pipesFakeOdo_->writeCommand(odo);;// We send fake data
    }
    {
        Odometry odo = populateOdoUAV(0,-2,0);//x=0, y=-2, yaw=0;
        pipesFakeOdo_->writeCommand(odo);;// We send fake data
    }

    //Set-up the controllers

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());

    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);
    controllers.at(1)->setTolerance(0.2);

    //Goals
    pfms::geometry_msgs::Point goal0{0,3};
    pfms::geometry_msgs::Point goal1{10,3};

    std::vector<pfms::geometry_msgs::Point*> goals;
    goals.push_back(&goal0);
    goals.push_back(&goal1);

    //Below now is an analysis of the platforms for each goal
    //Ackerman can not reach goal 0
    {

        bool reachable = controllers.at(0)->setGoal(goal0);
        EXPECT_FALSE(reachable);
    }

    //The Ackerman can reach goal 1
    {

        bool reachable = controllers.at(0)->setGoal(goal1);

        double dist = controllers.at(0)->distanceToGoal();

        double t = controllers.at(0)->timeToGoal();

        std::cout << "Ackerman: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
        EXPECT_NEAR(dist,10,0.5);
        EXPECT_NEAR(t,3.44,1.0);
    }


    //The Quadcopter can reach goal 0
    {
        bool reachable = controllers.at(1)->setGoal(goal0);
        double dist = controllers.at(1)->distanceToGoal();
        double t = controllers.at(1)->timeToGoal();
        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
        EXPECT_NEAR(dist,5,0.2);
        EXPECT_NEAR(t,12.5,0.5);
    }

    //The Quadcopter can reach goal 1
    {
        bool reachable = controllers.at(1)->setGoal(goal1);
        double dist = controllers.at(1)->distanceToGoal();
        double t = controllers.at(1)->timeToGoal();
        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
        EXPECT_NEAR(dist,11.1803,0.2);
        EXPECT_NEAR(t,27.9508,0.5);

    }


    //The Quadcopter can reach goal 1 from goal 0
    {
        Odometry odo = populateOdoUAV(goal0.x,goal0.y,0);//x=0, y=-2, yaw=0;

        double distance =0; double t=0;
        pfms::nav_msgs::Odometry estimateGoalPose;
        // Setting tolerance for goal (needed to unit test)
        bool reachable = controllers.at(1)->checkOriginToDestination(odo, goal1,distance,t,estimateGoalPose);

        std::cout << "Quadcopter: can reach goal " <<
                             distance << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
        EXPECT_NEAR(distance,10,0.2);
        EXPECT_NEAR(t,25,0.5);
        EXPECT_NEAR(estimateGoalPose.x,goal1.x,0.2);
        EXPECT_NEAR(estimateGoalPose.y,goal1.y,0.2);

    }

    //Let's set to begining
    {
        Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
        pipesFakeOdo_->writeCommand(odo);;// We send fake data
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
    }
    {
        Odometry odo = populateOdoUAV(0,-2,0);//x=0, y=-2, yaw=0;
        pipesFakeOdo_->writeCommand(odo);;// We send fake data
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
  
    }



    /////////////////////////////////////////////////////////////////////////////////
    // Some analysis here
    //
    // Goal 0 : Given Ackerman can not reach this goal the Quad will need to go there
    // Goal 1 : Eitehr Ackerman or Quad can reach it ...
    //
    // If objective is TIME then
    // Goal 0 : Platform 0 (Quadcopter)
    // Goal 1 : Platform 1 (Ackerman)

    // If objective is DISTANCE then
    // Goal 0 : Platform 0 (Quadcopter)
    // Goal 1 : Platform 1 (Quadcopter) - as the Quad will be closer to Goal 1 then Ackerman
    //                                    it would have visited Goal 0 as dist Goal 0 to Goal 1
    //                                    is shoter than Ackerman to Goal 1



    //////////////////////////////////////////////////////////////////////////////////
    // Let's now check missions
    Mission mission(controllers);
 
    mission.setGoals(goals);

    mission.setMissionObjective(mission::Objective::TIME);

    {
        std::vector<unsigned int> assignment =  mission.getPlatformGoalAssociation();

        for(unsigned int i=0;i<assignment.size();i++){
            std::cout << i << " : " << assignment.at(i) << std::endl;
        }

        ASSERT_EQ(assignment.size(),goals.size());//Need to have assigned a platform for each goal
        ASSERT_EQ(assignment.at(0),1); // Platform 1 should be going to goal 0
        ASSERT_EQ(assignment.at(1),0); // Platform 1 should be going to goal 1
    }

    mission.setMissionObjective(mission::Objective::DISTANCE);

    {
        std::vector<unsigned int> assignment =  mission.getPlatformGoalAssociation();

        for(unsigned int i=0;i<assignment.size();i++){
            std::cout << i << " : " << assignment.at(i) << std::endl;
        }

        ASSERT_EQ(assignment.size(),goals.size());//Need to have assigned a platform for each goal
        ASSERT_EQ(assignment.at(0),1); // Platform 1 should be going to goal 0
        ASSERT_EQ(assignment.at(1),1); // Platform 1 should be going to goal 1
    }

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
