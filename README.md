# ROS Beginner Tutorials - Publisher / Subcriber 


## Overview

This is a simple code meant to demonstrate the publisher and subscriber nodes in ROS. The talker node continuously publishes messages on the topic "chatter". The listener node subscribes to the said topic and receives the messages.

## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu (Wily/Xenial) or Debian (Jessie)
* ROS Kinetic

To instal ROS, use this [link](http://wiki.ros.org/kinetic/Installation)

## Build Instructions
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

## Run Instructions 

After following the build instructions:

Set environment vairables and run roscore in a new terminal:
```
source devel/setup.bash
roscore
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
