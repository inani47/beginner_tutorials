# ROS Tutorials - ROS TF, unit testing, bag files 


## Overview

These programs demonstrate the various concepts of ROS like publisher/subscriber nodes, services, tf and rostest. The talker node continuously publishes messages on the topic "chatter" as well as a constant tf transform. The listener node subscribes to the topic /chatter and receives the messages. There is a service called edit base string that can be used to modify a string within a program. All 5 logger levels are implemented in the code. The launch file included can be used to run all nodes and the service from a single command. It also records all messages being published on all topics in a bagfile. The code also uses gtest/rostest to implement a lever 2 integration test that tests the tf transform and the service of the talker node. 

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
roslaunch beginner_tutorials Week_11HW.launch 
```
To lower the publishing frequency using commind-line arguments:
Set environment variables and run talker node from a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials Week_11HW.launch pubRate:=1
```


## To Run service from command-line
This service takes in any give string and outputs a constant string.
To run the service, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /editBaseString "input: '<any_string>'" 
```

## To Inspect TF frames
First run the string editor service and talker node from the terminal from the catkin workspace (since talker waits for the service to start):
```
source devel/setup.bash
rosrun beginner_tutorials stringEditorService.cpp
rosrun beginner_tutorials talker
```

To run view_frames tool, from a new terminal:
```
rosrun tf view_frames
```
This will create a diagram of the frames being broadcast by tf over ROS in the folder from where the command was run.

To look at the transform being broadcast between the world frame and the talk frame. In a new terminal run the following command:
```
rosrun tf tf_echo world talk
```

## Running Rostest 
To run rostest:
```
cd ~/catkin_ws/build
make run_tests
```
This will build and run two tests which tests the service and the tf transform of the talker node.

## Recording and Playing bag files
To record bag files simply run the launch file from a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials Week_11HW.launch 
```

To disable bag file recording use the "rec" argument while running the launchfile:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials Week_11HW.launch rec:=1
```

To inspect the bag file go the folder where bag file was saved. For our convenience an example bag file is present in the results folder. In a new terminal
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/beginner_tutorials/results
rosbag info recording.bag
```

To playback the bag file and to verify that it is publishing on all the recorded topics.
First run the listener node on a new terminal:
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
Now from the results folder run the following command in the terminal:
```
rosbag play recording.bag
```
You will see that listener will start receiving messages being published on topic /chatter.





