Week 10 Tutorial Questions
=========================

Symbolically link the `topics_masterclass` package supplied in week10 starter packages (this assumes you have already linked pfms_support) to be part of the system as per [README.md](README.md)
```bash
cd ~/catkin_ws/src 
ln –s <your_git_repo>/tutorials/week10/starter/topics_masterclass.
```
* Build the package using the `catkin_make` command
* Run the simulation using `rosrun week10 week10-sample`

We will be modifying the `topics_masterclass` package in class.


Ex01: Find the closest point [x,y] to the robot in local coordinates (relative to robot) using sensor_msgs::LaserScan
-----------------------------------------------
See the `laserCallback` function of the `PfmsSample` class to complete this exercise.

 **Your task is to print the location of the nearest obstacle in the laser scan. The location should be given as the x, y position of the obstacle *relative* to the robot's coordinate frame.**

* On command line type `rosmsg show sensor_msgs/LaserScan`
* What are we provided in this message?
* Do we have the information in this message to find the closest point?
* What part of the message do we need to iterate over?
* How do we convert from range data to {x,y} [this is known as polar to cartesian](https://www.mathsisfun.com/polar-cartesian-coordinates.html)
* Where is the time of this message stored?
* Is the closest point identified the same as the one you see as closest on the stage simulator? Why is this the case?

Ex02 : Using an OccupancyGrid Map
-----------------------------------------------
Find the value of a cell relative to the robot using the [OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html). While the grid moves with the robot it does not change orientation (this is efficient computational use).  See the `occupancyGridCallback` function of the `PfmsSample` class to complete this exercise.

The occupancy grid is built such that the centre point of the map (width/2 and height/2) is at the location of the robot.

The value of each cell represents occupancy probability which is in the range [0,100].  Unknown is -1.
[nav_msgs/OccupancyGrid Message](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)

* Where are the metadata for the map stored (MetaData contains: width, height and resolution)
* How would we find any arbitrary x,y combining the map origin, the width, height and resolution?
```
     ^ y
     |
     |
     o------> x
```
* Print out the cell values at (x=-5, y=-5) relative to the centre? Is it occupied?

Ex03: OccupancyGrid and Robot Position
---------------------------

Find the closest occupied cell and its local coordinates (relative to robot) using the [OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html). We will need to transform the point we obtained in Ex02 using robot position (again, as the map does not rely on the orientation of the robot we do not need to use the robot orientation) 


* How should we search for the closest point to make it more effective?
* Should the closest point be identified the same as the one you see as closest on the stage simulator?


Ex04: Visualisation in RVIZ
---------------------------

We will use the  [MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html) to show items on rviz, it allows drawing arrows, axis, cylinders, squares, lines etc. 

Find the closest point in global coordinates {x,y} and publish it in MarkerArray, refer to the code for an example.

* We should either combine the information from pose / laser or pose / occupancy grid into this thread
* We use the visualisation marker code below (shall we put it in a function?)

Ex05: Subscribe to another topic
---------------------------

This is a strech goal for today, let's receive information directly from rviz into this node, receive a [ClickedPoint from RViz](https://answers.ros.org/question/69019/how-to-point-and-click-on-rviz-map-and-output-the-position/) into this node

* Find the topic name `/clicked_point` in the list of topics and determine its type
* Create a callback for this topic name and correct type
* Print out the sequence number and x , y coordinates using `ROS_INFO_STREAM`


[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
[Polar to Cartesian]: https://www.mathsisfun.com/polar-cartesian-coordinates.html
