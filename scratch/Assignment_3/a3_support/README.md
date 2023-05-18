A3 Support - Discussion
=========================

The support code can be used for either project

### Compiling

Before you get started, make sure you do the following link the `a3_support` folder to your catkin workspace, (ie if your path is <YOURGIT>/skeleton/a3_support)  then execute:

```bash
cd ~/catkin_ws/src
ln -s <YOURGIT>/skeleton/a3_support
```

Compile packages using `catkin_make` 

```bash
cd ~/catkin_ws
catkin_make
```

### Documentation

All documentation related to the package (running and anticipated behaviour) is provided in via doxygen.

To generate the documentation (There is a dot ''." after rosdoc_lite command, indicating the documentation is generated for that folder )

```bash
cd ~/catkin_ws/src/a3_support
rosdoc_lite .
```

You will find the documentation inside doc folder.

```bash
firefox ~/catkin_ws/src/a3_support/doc/html/index.html
```
