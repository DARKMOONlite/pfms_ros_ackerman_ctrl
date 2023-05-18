// The producer-consumer problem implemented with POSIX IPC
// Copyright Guido Klingbeil (2013)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// producer example that sends data of UGV
// does not manage shared memory, only writes to it

#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::geometry_msgs::Goal;

int main(int argc, char *argv[]) {

    if(argc !=4){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << "<seq_num> <goal_1_x> <goal_1_y>" << endl;
        return 0;
    }

    Pipes pipes(5,"/goal_buffer_seg","/goal_buffer_wsem","/goal_buffer_rsem");

    /* produce messages */
    Goal goal {static_cast<unsigned long>(atoi(argv[1])),
        {atof(argv[2]),atof(argv[3])}
         };
    pipes.writeCommand(goal);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
   return 0;
}
