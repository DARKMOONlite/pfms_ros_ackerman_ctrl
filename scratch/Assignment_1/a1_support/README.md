Assignment 1 - Setup
=========================

The below assumes you have installed ROS and created a workspace as per instructions under Week 04 module on canvas. In instructions `<YOUR_GIT>` is the location of your git repository. Mine is `~/git/pfms-2022a-alalemp` for instance.

We need to install a ros physics simulator (gazebo) which needs ~ 140MB download and will occupy 651MB of your disk space.

```bash
sudo apt-get update
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs ros-noetic-robot-state-publisher ros-noetic-tf2-ros libignition-math4-dev
```
Then install the pipes library, which has been supplied to allow using the physics simulator, at presnt bypassing the ROS framework for students.

```bash
cd <YOUR_GIT>/skeleton/a1_support/
sudo dpkg -i pipes-1.0.1-Linux.deb
```

Then link the `a1_ros` folder to your `catkin_ws/src`

```bash
cd ~/catkin_ws/src
ln -s <YOUR_GIT>/skeleton/a1_support/a1_ros/ 
```
Now we can make the package.

```bash
cd ~/catkin_ws
catkin_make
```

Compile the example code

```bash
cd <YOUR_GIT>/skeleton/a1_support/a1_snippets
mkdir build
cd build
cmake ..
make
````

You should now have all the required software. You can test the Audi or the Drone or both.

```
roslaunch gazebo_tf multi.launch
```
OR
```
roslaunch gazebo_tf ugv.launch
```
OR
```
roslaunch gazebo_tf uav.launch
```

You can use the example code to send commands to the platforms which is in `<YOUR_GIT>/skeleton/a1_support/a1_snippets/build`

For the Audi `./command_ugv <repeats> <brake> <throttle> <steering>`. For example.
```

```

For the Quadcopter `./command_uav <repeats> <turn_l_r> <move_l_r> <move_u_d> <move_f_b>"`
```
./producer_uav 10 0 0 0.7 0
```
There is also a gui for the quadcopter (just for sanity checking), hit `Z` first to enable control.

```bash
rosrun sjtu_drone drone_keyboard
```

