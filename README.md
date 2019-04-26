# Final Project: robot_generator package
- Automation of process for simulating a variety of configurable robots in Gazebo
- ME369P (Application Programming for Engineers)
- Keya Ghonasgi, Mincheol Kim, Matthew Times

## Package description
This package enables the user to generate a customized robot. The user can define the number of links, type of joints, and mass of the robot and simulate on Gazebo which is a physics-engine-based simulator. TO BE ADDED


## Required packages
This packages requires ROS and ros_control packages to run. 
[Check which ROS distribution is compatible with your version of Ubuntu.](http://wiki.ros.org/Distributions)
For Ubuntu 18.04 (melodic):
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-ros-controllers
```
For other versions of Ubuntu, simply change 'melodic' to a compatible distribution.

## Instructions for installing this ROS package:
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
Run simulation.launch for default demo. 
```
roslaunch robot_generator simulation.launch
```

## Walk-through of this package
TO BE ADDED

## Built With
* [ROS](http://www.ros.org/) - robot operating system

## Authors
* **Keya Ghonasgi** - task delegation
* **Mincheol Kim** - task delegation
* **Matthew Times** - task delegation
