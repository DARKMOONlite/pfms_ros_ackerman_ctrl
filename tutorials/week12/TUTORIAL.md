Week 12 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. 
If you have any questions about the material, please raise them in the next lab session.

This week's lab questions will make use of the pfms_support package provided to assist you with assignment 3 and well as assignment 1 material.

## Preasmble

Before you get started, make sure you do the following:

* Check out the latest code from the repository (fetch/merge)
* Link the `uav_skeleton` folder to your catkin workspace, (ie if your path is <YOURGIT>/tutorial/week12/starter/uav_skeleton then execute:
```bash
cd ~/catkin_ws/src
ln -s <YOURGIT>/tutorial/week12/starter/uav_skeleton
```

* Compile packages using `catkin_make`
```bash
cd ~/catkin_ws
catkin_make
```

* Compile tests using `catkin_make tests`  ... there is an *s* at the end of tests!
```bash
cd ~/catkin_ws
catkin_make tests
```

To stat the simulation run

```bash
roslaunch gazebo_tf uav_a3.launch
```

If you have any errors reposrted about models, you migth need to terminate roslaunch (may take time to close everything) execute below, and then repeat roslaunch.

```
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
```

If you wish to see the apperance of the  simulator you can execute `roslaunch gazebo_tf uav_a3.launch gui:=true` but this will exhaust almost all CPU on your computer, and therefore is not advisable, especilay if your running a Virtual Machine.

If the drone is not flying `rostopic pub /drone/takeoff std_msgs/Empty "{}"` OR you can open the gui to control it `rosrun sjtu_drone drone_keyboard` and then **HIT Z** to litf off.

if we execute `rostopic list` it will revela a numbe of topics associated with the drone. 

```bash
/drone/cmd_vel
/drone/gt_acc
/drone/gt_pose
/drone/gt_vel
/drone/imu
/drone/joint_states
/drone/land
/drone/posctrl
/drone/reset
/drone/scan
/drone/sonar/range
/drone/takeoff
/drone/vel_mode

```

We need `/drone/cmd_vel` to send the commands to drive the quadcopter. The senors are on `/drone/laser/scan` and `/drone/sonar/range`. We also need `/drone/takeoff` and `/drone/land` to tell the quadcopter to take off and land.

To check the topic message type  you can use rostopic, for example`rostopic info /drone/cmd_vel`  revelase the message type is `geometry_msgs/Twist`.

Ex01: Using LaserScan in ROS (Unit Testing)
----------------------------

Unit tests are generally performed on libraries, code that is well defined and is often isolated from the ROS framework of topics / services.  

In this case we have a [LaserProcessing](./starter/uav_skeleton/src/laserprocessing.h) class that is compiled into a library and thereafter used within the ROS framework.  Our TASK is to complete the function `detectPositionClosest` that determines the closest laser reading to the robot.

To check we have implemented it correctly, we run init tests against it. The uav_skeleton package defines  unit tests againt this library and compiles a test executable called `uav_skeleton_test`. You can find the unit tests itself as in [utest.cpp](./starter/services_masterclass/test/utest.cpp). 

 It is the nature of unit tests, that the answer is known for the test, checking the function for a variety of situations with known answers.  The test we have is done on a bag where we have saved laser data. Play the bag in ROS (make sure you have roscore running) `rosbag play -r 0.1 -l dome.bag`. Use rviz to inspect the data in this bag `rviz -d $(rospack find project_setup)/rviz/pfms.rviz `  and you can use the **Publish Point** feature in rviz and the `/clicked_point` topic to select what you belive to be the closest point and use it in the unit test (replacing the below with correct values).

```
  EXPECT_NEAR(point.x,3.39,0.1);
  EXPECT_NEAR(point.y,-1.89,0.1);
```

 Finally, use the `LaserProcessing` class inside the `Quadcopter`class. For the moment use the class inside the laserCallback function. 

Ex02: Threads in ROS node, combining information.
----------------------------

We will use the `seperateThread` function and our knowledge of mutexes to combine two pieces of information for a task that runs at a specific rate (1Hz in our case). We do not do this generally in callbacks, as they need to be very lightweight in terms of processing, and we can't slow down the processing to another desired rate.

We need to determine if we are detecting something/someone with the sonar that is pointing below the UAV. To do this, we will need to utilise the UAV position, stored as `odo_`and the readings from the sonar. To see the sonar readings `rostopic echo /drone/sonar/range`.  Contemplate how would you achieve this, we show how to use a mutex on odo_, you will need another mutex on the other data, and also to save the data as a member variable.  To experiment, move the quadcopter around using the keyboard `rosrun sjtu_drone drone_keyboard`. Go over a wall.

## Ex03: Using a Service

We have already advertised a service `/request_goals` with information on the service message `rossrv info project_setup/RequestGoal`. You can publish data to `/request_goals` from the terminal. 

```
rosservice call /request_goals "pose:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  poses:
  - position:
      x: 10.0
      y: 2.0
      z: 2.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0" 
```

Can you change the code to check if the goals are reachable, using current position of the Quadcopter and current laser reading. How would you do this? 

Implement the function that checks if the goals are within range of the laser (the goal is before the laser hits a wall). Add to the `LaserProcessing` class a function that accepts a goal as a Point and checks it against the laserScan stored in the class. Implement this check in `requestGoals` of `Quadcopter`. In the `Responce` field `geometry_msgs/PoseArray pose` store the points that are reachable. Also publish a marker at these points, you can use the `produceMarkerArray` function to do this. 

## Ex04: Publishing control commands to quadcopter

We need to modify our code to change the function `sendCmd` to send the actual commands to the quadcopter. Look at the topics, which one do we need to send to? We now need to change the constructor of `Quadcopter`to advertise we will send messages to this topic. Then we need to change `sendCmd` to send the actual commands. Note, we now alos need to deal with up/down of the quadcopter. 

As we have late submissions coming for Assignment 2 we can't disclose all the logic here yet in terms of threading and control. This is the Quiz4 material, so here you would have to call `reachGoal` which would be a blocking function (and it could  prevent callbacks from being invoked). Therefore, could you create a thread here to control the quadcopter is there were goals received from a service?




[services_masterclass]: starter/services_masterclass
[utest.cpp]: starter/services_masterclass/test/utest.cpp
[GridProcessing]: starter/services_masterclass/grid_processing.h
[quiz5a]: ../../quizzes/quiz5/a
[pfms_support]: ../../skeleton/pfms_support
