## A2 Unit Tests

If not done already

```bash
git fetch origin subject
git merge origin/subject --allow-unrelated-histories
git push
```

Copy the test folder into your current assignment 1 folder. For instance my assignment is in `~/git/pfms-2022a-alalemp/scratch/a2_skeleton` . That is where I've worked on my code.

```
cd ~/git/pfms-2022a-alalemp/skeleton/a2_skeleton
cp -rv test .. ~/git/pfms-2022a-alalemp/scratch/a2_skeleton
```

Once I copy **test** folder across it will look like.

```
student@lucani:~/git/pfms-2022a-alalemp/scratch/a2_skeleton$ ls
ackerman.cpp  build                       CMakeLists.txt       controller.cpp           controller.h           index.dox   logger.h  mission.cpp  missioninterface.h           quadcopter.cpp  test  ackerman.h    controllerinterface.h  logger.cpp  main.cpp  mission.h      quadcopter.h
```

## Compiling / Usage

Unlike the tests for quizzes, the test folder we have supplied is self-contained. You need to be in that folder to create a build directory and subsequently runs your tests. Mine is `~/git/pfms-2022a-alalemp/scratch/a2_skeleton/test`

Create a build directory inside the test directory and then do cmake

```bash
mkdir build
cd build
cmake ..
```

At this point, it depends if you have finished all your work, or are still working on some aspects. You can make just some of the tests OR all of them, so select from **either** of below, the last option `make` will make all 6 tests (3 for audi and 3 for quadcopter). **EVERY TIME you change your code, and you want to test, you will need to recompile the tests (run the make command).** You can also make individual tests as per below (select which test to make)

```
make audiCheckOriginToDestination
make audiReachGoal
make audiReachGoals
make quadcopterCheckOriginToDestination
make quadcopterReachGoal
make quadcopterReachGoals
```

For all of the tests **you have to use** `roslaunch gazebo_tf multi.launch`.  After teh tests CTRL+C the launch file and restart it.

```bash
roslaunch gazebo_tf a1_test.launch
```

## EXAMPLE OF AUD TESTS

Run the following command from within the build directory (mine is `~/git/pfms-2022a-alalemp/scratch/a1_skeleton/test/build`)
```
./audiCheckOriginToDestination
```

There is 1 test case group `Ackerman` with one test: `checkOriginToDestination`, though it will check this function for 10 points

```
student@lucani:~/git/pfms/assignments/a1/a1_example/test/build$ ./audiTests
./audicheckOriginToDestination
[==========] Running 1 test from 1 test case.
[----------] Global test environment set-up.
[----------] 1 test from Ackerman
[ RUN      ] Ackerman.checkOriginToDestination
Setting Ackerman Position  DONE
[       OK ] Ackerman.checkOriginT (6398 ms)
[----------] 1 test from Ackerman (6398 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test case ran. (6399 ms total)
[  PASSED  ] 1 test.
```
