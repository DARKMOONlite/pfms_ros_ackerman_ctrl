#include "rangeprocessing.h"
#include <algorithm>
#include <numeric>

namespace range_processing {


    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    bool detectHeightOfObject(sensor_msgs::Range rangeScan, geometry_msgs::Pose pose, double& height)
    {
        
        if(!(rangeScan.range <= rangeScan.min_range || rangeScan.range >= rangeScan.max_range)){
              height = pose.position.z - rangeScan.range;  
              return true;
        }
        return false;
    }


    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    bool detectPerson(sensor_msgs::Range rangeScan, geometry_msgs::Pose pose, double personHeight, geometry_msgs::Point& location)
    {
            std::cout << "min_range: " << rangeScan.min_range << " max_range: " << rangeScan.max_range <<std::endl;
            std:: cout << " Range: " << rangeScan.range << std::endl;
            std::cout << "Pose x:" << pose.position.x << " y:" <<pose.position.y << "z: " << pose.position.z << std::endl;

            double height =0;
            if(detectHeightOfObject(rangeScan,pose,height)){
                
                if(height > personHeight-0.2 && height < personHeight +0.2){ //Check range of heights near known person height to ensure its a person
                location.x = pose.position.x;
                location.y = pose.position.y;
                location.z = height;
                return true;
                }
                else{
                    return false;
                }


            }
            else{
                return false;
            }


            
    }


}
