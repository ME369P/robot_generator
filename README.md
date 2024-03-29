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
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-ros-controllers
```
For other versions of Ubuntu, simply change 'melodic' to a compatible distribution.

## Installation
Download this package into (your catkin workspace)/src folder.
```
$ mkdir ~/catkin_ws/ ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/ME369P/robot_generator
```
Go to (your catkin workspace), call catkin_make and source the catkin workspace.
```
$ cd ~/catkin_ws
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Walk-through
Run python in the scripts folder inside robot_generator package.
```
$ cd ~/catkin_ws/src/robot_generator/scripts
$ python3
```
Import robot_class and define a customized robot. Below is an example of a 2-DOF robot named "Groot".
```
>>> from robot_class import robot_class as rc
>>> robot = rc('Groot',2)
```
Enter link lengths and their density. Below is when you want each link to be length of 1 and density of 1.
```
Enter length of link #1: 1
Enter length of link #2: 1
Enter density of links: 1
```
Generate the robot.
```
>>> robot.create_robot()
```
In a different terminal, run simulation.launch on command terminal to simulate the generated robot.
```
$ roslaunch robot_generator {robot name}_simulation.launch
```

You can use tab completion to find all the launch files in the robot_generator directory. If you do not remember the name of the robot, in the Python terminal, call the method getID.
```
>>> robot.getID()
'Groot'
```

## Application demo (supports only revolute joints at the moment)
In a new terminal, run IK_demo.py and enter the name of the robot.
```
$ rosrun robot_generator IK_demo.py
Enter the name of the robot: 'Groot'
```
In a new terminal, run key_teleop.
```
$ rosrun robot_generator key_teleop.py
```
Use the arrow keys to control the robot end-effector on Gazebo. Make sure the key_teleop terminal is selected.

## References
* [ROS](http://www.ros.org/) - The robot operating system
* [ET](https://docs.python.org/2/library/xml.etree.elementtree.html) - The ElementTree XML API
* [Gazebo](http://gazebosim.org/) - Gazebo simulator
* [key_Teleop](http://wiki.ros.org/key_teleop) - package that reads keyboard inputs from the user

## Authors
* **Keya Ghonasgi** - Robot generator script
* **Mincheol Kim** - Teleoperation + Inverse kinematics application
* **Matthew Times** - ROS integration
