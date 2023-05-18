#include "analysis.h"
#include "tf.h"
#include "tf2.h"

using std::vector;
using std::pair;
using geometry_msgs::Point;

Analysis::Analysis(std::vector<Point> goals) :
    goals_(goals)
{

}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
vector<double> Analysis::timeToImpact(Pose origin){
    vector<double> times;
    for (int i = 0; i <goals_.size(); i++){
        RangeBearingStamped relative = tf2::global2local(goals_.at(i), origin);

        double rot_time = abs(tf2::normaliseAngle(relative.bearing)) / Display::OMEGA_MAX;
    //    std::cout << "Angle : "<< tf2::normaliseAngle(relative.bearing) << std::endl;
    //    std::cout <<  "Yaw : " <<  tf::quaternionToYaw(origin.orientation)  << std::endl;
        std::cout << "rotation: " << rot_time << std::endl;
        double trans_time = relative.range / Display::V_MAX;
        std::cout << "translation: " << trans_time << std::endl;
        times.push_back(rot_time+trans_time);
    }
    //The consts you will need are
    
    //Display::OMEGA_MAX Max angular velocity (rad/s)
    //Display::V_MAX Max velocity (m/s)

    
    return times;
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
AdjacencyList Analysis::exportGraph(){
      AdjacencyList graph;

    for(int i =0; i < goals_.size(); i ++){ // for each node of our graph
    
        vector<EdgeInfo> connections;
        pair<double, double> i_pos =  {goals_.at(i).x, goals_.at(i).y};

        for(int j =0; j < goals_.size(); j++){ //Create a connection from this node to each of the other nodes.
          
            if(j!= i){ //dont create connection between current node and itself.
                EdgeInfo result;
                pair<double, double> j_pos =  {goals_.at(j).x, goals_.at(j).y};
                double distance = sqrt(pow(i_pos.first-j_pos.first, 2) + pow(i_pos.second-j_pos.second, 2));
                result.first = distance;
                result.second = j;
                connections.push_back(result);

            }
        }
        graph.push_back(connections);
    }

    //std::make_pair(odom.at(i), i));
  

    return graph;
}
 