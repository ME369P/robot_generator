# Final Project: robot_generator package
- Automation of process for simulating a variety of configurable robots in Gazebo
- ME369P (Application Programming for Engineers)
- Keya Ghonasgi, Mincheol Kim, Matthew Times

## Required packages
This packages requires ROS and ros_control packages to run. Make sure you change 'melodic' to your version of Ubuntu:
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-ros-controllers
```


## Instructions for running this ROS package:

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
Run simulation.launch for default demo based on tutorials. 
```
roslaunch robot_generator simulation.launch
```

## Built With

* [ROS](http://www.ros.org/) - robot operating system

## Authors

* **Keya Ghonasgi** - task delegation
* **Mincheol Kim** - task delegation
* **Matthew Times** - task delegation
