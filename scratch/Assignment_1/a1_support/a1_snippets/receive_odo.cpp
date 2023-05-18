// consumer - the GPU simulation process
// does not manage shared memory, only write to it

#include "pipes.h"
#include <vector>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {

    if(argc !=2){
        std::cout << " Not arguments given on command line." << std::endl;
        std::cout << " usage: " << argv[0] << " <repeats>" << std::endl;
        return 0;
    }

    //! Created a pointer to data processing
    std::shared_ptr<Pipes> pipesPtr(new Pipes(8,"/uav_odo_buffer_seg","/uav_odo_buffer_wsem","/uav_odo_buffer_rsem",true));

     for (unsigned int i=0;i<atoi(argv[1]); i++){
         pfms::nav_msgs::Odometry odo =  pipesPtr->getOdo();
         std::cout << "i seq x,y,yaw,vx,vy: " <<
                  i << " " <<
                  odo.seq << " " <<
                  odo.x << " " <<
                  odo.y << " " <<
                  odo.yaw << " " <<
                  odo.vx << " " <<
                  odo.vy << std::endl;
         std::this_thread::sleep_for (std::chrono::milliseconds(100));
     }

    return 0;
}
