## A1 Unit Tests

If not done already 

```bash
git fetch origin subject
git merge origin/subject --allow-unrelated-histories
git push
```

Copy the test folder into your current assignment 1 folder. For instance my assignment is in `~/git/pfms-2022a-alalemp/scratch/a1_skeleton` . That is where I've worked on my code.

```
cd ~/git/pfms-2022a-alalemp/skeleton/a1_skeleton
cp -rv test .. ~/git/pfms-2022a-alalemp/scratch/a1_skeleton
```

Once I copy **test** folder across it will look like.

```
student@lucani:~/git/pfms-2022a-alalemp/scratch/a1_skeleton$ ls 
ackerman.cpp    CMakeLists.txt.user    index.dox    missioninterface.h  test
ackerman.h      controller.cpp         main.cpp     quadcopter.cpp
build           controller.h           mission.cpp  quadcopter.h
CMakeLists.txt  controllerinterface.h  mission.h    README.md
```

We have a specific launch file for unit testing. Therefore you do need to update your catkin workspace

```
cd ~/catkin_ws
catkin_make
```

## Compiling / Usage

Unlike the tests for quizzes, the test folder we have supplied is self-contained. You need to be in that folder to create a build directory and subsequently runs your tests. Mine is `~/git/pfms-2022a-alalemp/scratch/a1_skeleton/test`

Create a build directory inside the test directory and then do cmake

```bash
mkdir build
cd build
cmake ..
```

At this point, it depends if you have finished all your work, or are still working on some aspects. You can make just some of the tests OR all of them, so select from **either** of below, the last option `make` will make all 3 tests. **EVERY TIME you change your code, and you want to test, you will need to recompile the tests (run the make command).**

```
make audiTests
make quadcopterTests
make missionTests
make 
```

For all of the tests **you have to use** `roslaunch gazebo_tf a1_test.launch` . You have to use this launch file for the unit tests, **you can not use** `multi.launch` or `ugv.launch` or `uav.launch`. 

```bash
roslaunch gazebo_tf a1_test.launch
```

## AUDI TESTS

Run the following command from within the build directory (mine is `~/git/pfms-2022a-alalemp/scratch/a1_skeleton/test/build`)
```
./audiTests
```

There is 1 test case group `AkermanTest` with four tests: `Simple, Simple2, NotPossible,ForwardFacing`

```
student@lucani:~/git/pfms/assignments/a1/a1_example/test/build$ ./audiTests 
[==========] Running 4 tests from 1 test case.
[----------] Global test environment set-up.
[----------] 4 tests from AckermanTest
[ RUN      ] AckermanTest.Simple
Wrote buffer
Turning on threading
readOdo
Ackerman: can reach goal 10.2646[m] 3.53951[s]
[       OK ] AckermanTest.Simple (110 ms)
[ RUN      ] AckermanTest.Simple2
Wrote buffer
Turning on threading
readOdo
Ackerman: can reach goal 13.3518[m] 4.60406[s]
[       OK ] AckermanTest.Simple2 (120 ms)
[ RUN      ] AckermanTest.NotPossible
Wrote buffer
Turning on threading
readOdo
GOAL [x,y]:0,-6
[       OK ] AckermanTest.NotPossible (160 ms)
[ RUN      ] AckermanTest.BackwardFacing
Wrote buffer
Turning on threading
readOdo
Ackerman: can reach goal 10.2568[m] 3.53684[s]
[       OK ] AckermanTest.ForwardFacing (200 ms)
[----------] 4 tests from AckermanTest (590 ms total)

[----------] Global test environment tear-down
[==========] 4 tests from 1 test case ran. (590 ms total)
[  PASSED  ] 4 tests.
```



## Quadcopter Tests

Run the following command from within the build directory (mine is `~/git/pfms-2022a-alalemp/scratch/a1_skeleton/test/build`)

```
./quadcopterTests
```

There is 1 test case group `QuadcopterTest` with four tests: `Simple, Simple2, ForwardFacing,BackwardFacing`

```
[==========] Running 4 tests from 1 test case.
[----------] Global test environment set-up.
[----------] 4 tests from QuadcopterTest
[ RUN      ] QuadcopterTest.Simple
Wrote buffer
Turning on threading
readOdo
Quadcopter: can reach goal 10[m] 25[s]
[       OK ] QuadcopterTest.Simple (117 ms)
[ RUN      ] QuadcopterTest.Simple2
Wrote buffer
Turning on threading
readOdo
Quadcopter: can reach goal 6.5[m] 16.25[s]
[       OK ] QuadcopterTest.Simple2 (120 ms)
[ RUN      ] QuadcopterTest.ForwardFacing
Wrote buffer
Turning on threading
readOdo
Quadcopter: can reach goal 9.33822[m] 23.3455[s]
[       OK ] QuadcopterTest.ForwardFacing (160 ms)
[ RUN      ] QuadcopterTest.BackwardFacing
Wrote buffer
Turning on threading
readOdo
Quadcopter: can reach goal 14.1485[m] 35.3712[s]
[       OK ] QuadcopterTest.BackwardFacing (200 ms)
[----------] 4 tests from QuadcopterTest (597 ms total)

[----------] Global test environment tear-down
[==========] 4 tests from 1 test case ran. (598 ms total)
[  PASSED  ] 4 tests.
```

## Mission Tests

Run the following command from within the build directory (mine is `~/git/pfms-2022a-alalemp/scratch/a1_skeleton/test/build`)

```
./missionTests
```

There is 1 test case group `MissionTest` with a test that has two conditions DISTANCE and TIME called : `Simple`

```bash
[==========] Running 1 test from 1 test case.
[----------] Global test environment set-up.
[----------] 1 test from MissionTest
[ RUN      ] MissionTest.Simple
Wrote buffer
Wrote buffer
Turning on threading
readOdo
Turning on threading
readOdo
Ackerman: can reach goal 10.0665[m] 3.47122[s]
Quadcopter: can reach goal 5[m] 12.5[s]
Quadcopter: can reach goal 11.1803[m] 27.9508[s]
Quadcopter: can reach goal 10[m] 25[s]
0 : 1
1 : 0
0 : 1
1 : 1
[       OK ] MissionTest.Simple (854 ms)
[----------] 1 test from MissionTest (854 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test case ran. (855 ms total)
[  PASSED  ] 1 test.

```

