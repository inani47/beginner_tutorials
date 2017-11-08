# ROS Tutorials - Services, Logging and Launch Files 


## Overview

This is a simple code meant to demonstrate the publisher and subscriber nodes in ROS. The talker node continuously publishes 100 messages on the topic "chatter" and stops. The listener node subscribes to the said topic and receives the messages. There is a service called edit base string that can be used to modify a string within a program. All 5 logger levels are implemented in the code. The launch file included can be used to run all nodes and the service from a single command.

## License

This software is released under the BSD-2 License, see [LICENSE.txt](LICENSE.txt).

## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu (Wily/Xenial) or Debian (Jessie)
* ROS Kinetic

To instal ROS, use this [link](http://wiki.ros.org/kinetic/Installation)

## Build Instructions
If you do not have a catkin workspace, follow this:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/beginner_tutorials.git
cd ..
catkin_make
```
If you wish to run this code in an existing catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/beginner_tutorials.git
cd ..
catkin_make
```
## Run Instructions 

After following the build instructions:

Set environment vairables and run roscore in a new terminal:
```
source devel/setup.bash
roscore
```
Set environment variables and first run string editor service node from a new terminal:
```
source devel/setup.bash
rosrun beginner_tutorials stringEditorService.cpp
```
Set environment variables and run talker node from a new terminal:
```
source devel/setup.bash
rosrun beginner_tutorials talker
```
Set environment variables and run listener node from a new terminal:
```
source devel/setup.bash
rosrun beginner_tutorials listener
```

## Run Instructions Using Launch File

After following the build instructions:

In a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials Week_10HW.launch 
```
To lower the publishing frequency using commind-line arguments:
Set environment variables and run talker node from a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials Week_10HW.launch pubRate:=1
```
Note: Due to grouping in launch files, sometimes the service node may start after the other nodes. In this case, the string won't be outputted, only numbers. Using "Run Instructions" in the above section always gives the correct output.

## To Run service from command-line
In a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /editBaseString "input: '<any_string>'" 
```

