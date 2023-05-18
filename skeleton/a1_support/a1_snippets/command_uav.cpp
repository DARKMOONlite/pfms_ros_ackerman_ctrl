#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::UAV;

int main(int argc, char *argv[]) {

    if(argc !=6){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << "<repeats> <turn_l_r> <move_l_r> <move_u_d> <move_f_b>" << endl;
        return 0;
    }

    Pipes pipes(6,"/uav_buffer_seg","/uav_buffer_wsem","/uav_buffer_rsem");

    unsigned long i = 0;
    /* produce messages */
    for(i = 0; i < atoi(argv[1]); i ++) {
        UAV uav {i,atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5])};
        pipes.writeCommand(uav);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        cout << "wrote msg:" << i << endl;
    }
    UAV uav {i,0,0,0,0};
    pipes.writeCommand(uav);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    cout << "wrote msg:" << i << endl;

    return 0;
}
