ROS Beginner - Publisher / Subcriber 


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
git clone --recursive https://github.com/karanvivekbhargava/beginner_tutorials.git
cd ..
catkin_make
```

## Run Instructions 

After following the build instructions:

Run roscore from the terminal:
```
roscore
```
Run talker node from the terminal:
```
rosrun beginner_tutorials talker
```
Run listener node from the terminal:
```
rosrun beginner_tutorials listener
```
