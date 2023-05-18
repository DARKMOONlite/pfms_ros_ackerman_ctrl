#ifndef MISSIONINTERFACE_H
#define MISSIONINTERFACE_H

#include <vector>
#include "controllerinterface.h"
#include "ros/ros.h"



/**
 * @brief Specifies the required interface for your Mission class
 * must inherit from it. <b> You MUST NOT edit this file </b>.
 * 
 */

/*!
 *  \brief     Mision Interface Class
 *  \details
 *  Specifies the required interface for your Mission
 *  must inherit from it. <b> You MUST NOT edit this file </b>.
 *  \author    Alen Alempijevic
 *  \version   1.01-2
 *  \date      2022-03-02
 *  \pre       none
 *  \bug       none reported as of 2022-03-02
 *  \warning   students MUST NOT change this class (the header file)
 */

namespace  mission{

    typedef enum {
      TIME, /*!< Selection of Mission Objective based on Time */
      DISTANCE, /*!< Selection of Mission Objective based on Distance */
      BASIC, /*!< BASIC mode of mission, execution of goals in order supplied */
      ADVANCED /*!< ADVANCED mode of mission, evaluated best order of goals minimising DISTANCE */
    } Objective; /*!< Objective Types */

}


class MissionInterface
{
public:
    MissionInterface(){};

    /**
     * @brief Accepts the container of goals.
     *
     * @param goals
     */
    virtual void setGoals(std::vector<std::pair<pfms::geometry_msgs::Point,pfms::geometry_msgs::Point> > cones) = 0;

    /**
     * @brief Runs the mission, non blocking call
      * @return bool indicating mission can be completed (false if mission not possible)
     */
    virtual bool run() = 0;

    /**
    Retrurns mission completion status (indicating percentage of completion of task) by each platform @sa setGoals
    @return vector with each element of vector corresponding to a platform. The value is percent of completed distance of entire mission for the corresponding platform value between 0-100.
    */
    virtual std::vector<unsigned int> status(void) = 0;


    /**
     * @brief Set mission objective
     */
    virtual void setMissionObjective(mission::Objective objective) = 0;

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the total distance travelled by the corresponding platform
     * from the time of starting the program.
     *
     * @return std::vector<double> - each element is distance travelled for each platform [m]
     *
     */
    virtual std::vector<double> getDistanceTravelled() = 0;

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the time the corresponding platfore has been moving
     * from the time the program started. Moving means the platform was not stationary.
     *
     * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
     *
     */
    virtual std::vector<double> getTimeMoving() = 0;




};

#endif // MISSIONINTERFACE_H
