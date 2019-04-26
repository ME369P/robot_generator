# Final Project: robot_generator package
- Automation of process for simulating a variety of configurable robots in Gazebo
- ME369P (Application Programming for Engineers)
- Keya Ghonasgi, Mincheol Kim, Matthew Times

## Package description
This package enables the user to generate a customized robot. User can define the number of links, type of joints, and mass of the robot and simulate on Gazebo which is a physics-engine-based simulator. User can also run a python script to control the robot. As an example, the default application demonstrates a keyboard-controlled n-DOF manipulator. 

## Required packages
This packages requires ROS and ros_control packages to run. 
[Check which ROS distribution is compatible with your version of Ubuntu.](http://wiki.ros.org/Distributions)
For Ubuntu 18.04 (melodic):
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-ros-controllers
```
For other versions of Ubuntu, simply change 'melodic' to a compatible distribution.

## Installation
Download this package into (your catkin workspace)/src folder.
```
mkdir ~/catkin_ws/ ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ME369P/robot_generator
```
Go to (your catkin workspace), call catkin_make and source the catkin workspace.
```
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```


## Walk-through
How to parse robots
```
how to parse robots
```
Run simulation.launch to simulate the generated robot.
```
roslaunch robot_generator simulation.launch
```

## Application demo
Install key_teleop package to communicate with the robot through keyboard strokes.
```
sudo apt-get install key_teleop
```
With the simulation running, run key_teleop.
```
rosrun key_teleop key_teleop.py
```
run RRbot_IK_demo.py.
```
rosrun robot_generator RRbot_IK_demo.py
```
On key_teleop window, try pressing arrow keys to control the robot end-effector on Gazebo.

## References
* [ROS](http://www.ros.org/) - The robot operating system
* [ET](https://docs.python.org/2/library/xml.etree.elementtree.html) - The ElementTree XML API

## Authors
* **Keya Ghonasgi** - task delegation
* **Mincheol Kim** - task delegation
* **Matthew Times** - task delegation
