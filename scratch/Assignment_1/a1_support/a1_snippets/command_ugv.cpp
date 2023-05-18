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
using pfms::commands::UGV;

int main(int argc, char *argv[]) {

    if(argc !=5){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << "<repeats> <brake> <throttle> <steering>" << endl;
        return 0;
    }

    Pipes pipes(5,"/ugv_buffer_seg","/ugv_buffer_wsem","/ugv_buffer_rsem");

    unsigned long i = 0;
    /* produce messages */
    for(i = 0; i < atoi(argv[1]); i ++) {
        UGV ugv {i,atof(argv[2]),atof(argv[3]),atof(argv[4])};
        pipes.writeCommand(ugv);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cout << "wrote:" << i << endl;
    }

   return 0;
}
