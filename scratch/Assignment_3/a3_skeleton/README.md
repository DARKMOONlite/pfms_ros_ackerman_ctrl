A3 Skeleton - Discussion
=========================

The skeleton code can be used for either project, consider this document when making changes to the code for your project.

### Compiling

Before you get started, make sure you do the following:

* Check out the latest code from the repository
* Link the `a3_skeleton` folder to your catkin workspace, (ie if your path is <YOURGIT>/tutorial/skeleton/a3_skeleton then execute:
```bash
cd ~/catkin_ws/src
ln -s <YOURGIT>/skeleton/a3_skeleton
```

* Compile packages using `catkin_make` and compile tests using `catkin_make tests`
```bash
cd ~/catkin_ws
catkin_make
```

* Compile tests using `catkin_make tests`  ... there is an *s* at the end of tests!
```bash
cd ~/catkin_ws
catkin_make tests
```

### Execution

You can run the code in UAV or UGV mode. In current form the code requies a parameter to use/control UAV or UGV.

**For UGV (Ackerman - AUDI R8)**

Run the below commands in 3 terminals

```bash
roscore
roslaunch gazebo_tf ugv_a3.launch
rosrun a3_skeleton a3_skeleton_sample _is_ugv:=true
```

The code:

* Subscribes to some topics / Advertises some topics / Advertises a service
* Has a seperate thread of execution where it sends steering 1.0 rad, throttle 0.1 every 0.2s 
* Reports closest point in laser (in vehicle coordinate frame : *orange/front_laser_link* ) at every laser reading received
* Publishes a visualisation marker (cylinder) under name-space test  using current robot location (adding 1m to x,y,z or current odometry)
* Uses a mutex to protect data modified in callback and used in seperate thread

**For UAV (Drone - Quadcopter)**

Run the below commands in 3 terminals

```bash
roscore
roslaunch gazebo_tf uav_a3.launch
rosrun a3_skeleton a3_skeleton_sample _is_ugv:=false
```

The code:

* Subscribes to some topics / Advertises some topics / Advertises a service
* Has a seperate thread of execution where it sends velocity to control the quadcopter
* Reports closest point in laser (in sensor coordinate frame : *drone/hokuyo_link* ) at every laser reading received
* Publishes a visualisation marker (cylinder) under name-space test  using current robot location (adding 1m to x,y,z or current odometry)
* Uses a mutex to protect data modified in callback and used in seperate thread

### **Moving forward you need to examine the project specifications and make changes**

Make changes to the skeleton, DO NOT LEAVE obsolete or unused function  calls.

You will need to determine which topics to subsribe to (callbacks to enable) as well as which topics to advertise (data to publish).  There are some in-code comments that indicate how to proceed with this. 

Examine example of obtaining parameters on the command line (a bool and a double), you need to remove or adapt this to suit your project. 

Add more threads if required, it is best to decouple sensing (decision making) and control.

Examine the service call and make changes to what it accomplishes

Use your knowledge of classes / inheritance / access specifiers / threading / data protection, all aspects covered thus far to modify sample code. 

### **Unit testing**

We do not provide data or specific code for the unit testing in the final assignment, you will need  to develop and test your own code using the knowledge gained from the subject. Unit tests are generally performed on libraries.  **You should NOT have code with callbacks being tested**. You can opt to have different constructors that allow various forms of integration. It is the nature of unit tests, that the answer is known for the test, checking the function for a variety of situations with known answers.  

The unit test in this skeleton uses a rosbag that has been previously recorded, for which an answer of closest point is known (observed from simulation) and this is used.  You will need to make changes to the code for your assignment, recording a suitable bag that examines your node, where you INDEPENDENTLY assess the solution and then check your code performance.  REVIEW weeks 11/12 about the tools in rviz that let you obtain data that can be used in the unity test. The content in week 11/12 also shows how to record a rosbag that can be used in the unit test, and this was an activity in class in those weeks.

In the unit test the rosbag is opened, and one reading each for position of robot (odometry) and laser scan is used. A function of a class is developed and tested for. NOTE that we here need to specify what topic name and data type we are looking for in the bag. 

The package defines some unit tests and compiles a test executable called `a3_skeleton_test`. You can find the unit tests itself as in [utest.cpp](./starter/services_masterclass/test/utest.cpp) You can run tests 

```bash
rosrun a3_skeleton a3_skeleton_test
```

To record a rosbag that you can use in unit testing, you can use the rosbag utility. You can record any of the topics with syntax below (-l 10 records 10 messages). Don't forget to record the /tf and /tf_static messages as these are needed for rviz to show data when your playing back the bag.

```bash
rosbag record -l 10 /topic_name1 /topic_name2 .... /topic_name3 
```

For instance

```
rosbag record -l 10 /uav_odom /drone/laser/scan /drone/sonar/range /drone/cmd_vel /tf /tf_static
```

Records a bag with current timestapmp `YYYY-MM-DD-HH-MM-SS.bag`. Playing the bag back (example below has bad called `2022-05-23-11-23-47.bag` is below (it replays at 0.1 original rate `-r 0.1`, using the clock at time of recording `--clock` and does so in loop `-l`

```
rosbag play -r 0.1 --clock -l 2022-05-23-11-23-47.bag
```

### Documentation

In source documentation is a must, we will examine all code submitted for supporting documentation. You also need the dox file (mainpage.dox) to indicate how the code will run/behave. You need to modiy the mainpage.dox included which does not have any specific documentation.

In ROS, doxygen documentation in generated using the `rosdoc_lite` tool. If you do not have the tool you can install it via `sudo apt-get install ros-noetic-rosdoc-lite` (replace noetic with melodic if on 18.04)

To generate the documentation'

```bash
cd ~/catkin_ws/src/a3_skeleton
rosdoc_lite .
```

You will find the documentation inside doc folder.

```bash
firefox ~/catkin_ws/src/a3_skeleton/doc/html/index.html
```






[services_masterclass]: starter/services_masterclass
[utest.cpp]: starter/services_masterclass/test/utest.cpp
[GridProcessing]: starter/services_masterclass/grid_processing.h
[quiz5a]: ../../quizzes/quiz5/a
[pfms_support]: ../../skeleton/pfms_support
