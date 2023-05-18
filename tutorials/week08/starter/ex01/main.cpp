#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "quadcopter.h"
#include "ackerman.h"
#include "mission.h"
#include "a1_types.h"

//using namespace std;
using namespace pfms::nav_msgs;

//! User-provided mission data
struct MissionInfo {
    //! x,y coordinates of goals
    std::vector<pfms::geometry_msgs::Point> points;
    //! Objective to optimise for
    mission::Objective objective;
};

MissionInfo getMissionFromUser() {
    MissionInfo res;
    int num_points;
    std::cout << "Number of goals? ";
    std::cin >> num_points;
    res.points.reserve(num_points);

    for (int i = 0; i < num_points; i++) {
        pfms::geometry_msgs::Point pt;
        std::cout << "\nGoal " << i << " x? ";
        std::cin >> pt.x;
        std::cout << "\nGoal " << i << " y? ";
        std::cin >> pt.y;
        res.points.push_back(pt);
    }

    std::cout << "\nObjective? ";
    int objective_tmp;
    std::cin >> objective_tmp;
    switch (objective_tmp) {
        case 0:
        default:
            res.objective = mission::Objective::DISTANCE;
            break;
        case 1:
            res.objective = mission::Objective::TIME;
            break;
    }
    std::cout << "\n";

    return res;
}

int main() {
    // Get user inputs
    auto mission_info = getMissionFromUser();

    //Set-up the controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());

    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);
    controllers.at(1)->setTolerance(0.2);

    //Goals
    Pipes goal_pipe(5,"/goal_buffer_seg","/goal_buffer_wsem","/goal_buffer_rsem");
    unsigned long seq = 0;

    while (true) {
        for (auto pt : mission_info.points) {
            pfms::geometry_msgs::Goal goal{seq++, pt};
            goal_pipe.writeCommand(goal);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::vector<pfms::geometry_msgs::Point*> goals;
        goals.reserve(mission_info.points.size());
        for (auto& pt : mission_info.points) {
            goals.push_back(&pt);
        }

        Mission mission(controllers);
        mission.setGoals(goals);

        mission.setMissionObjective(mission_info.objective);
        mission.runMission();

        std::cout << "New mission? [Y/N] ";
        char response;
        std::cin >> response;
        if (tolower(response) != 'y') break;

        mission_info = getMissionFromUser();
    }
}
