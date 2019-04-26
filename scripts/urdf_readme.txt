Readme file for robot_class.py

To use with python (this is an Eg.):
    - run python in the same folder as this file.
    - from robot_class import robot_class
    - robot = robot_class('name',5)
    - Enter the link lengths, types of joints (-1 for prismatic, 0 for fixed, 1 for revolute continuous), orientation for fixed joints and densite of the links (used to calculate mass and inertia). The width and thickness are set in the robot_class function write_urdf
    - robot.write_urdf()

This will create a urdf file for the robot you described. 

    - In another terminal, launch roscore
    - In another terminal, enter the following:
    - roslaunch urdf_tutorial display.launch model:=robot_name.urdf

A few things to note:

The urdf field is named using the 'name' you pass as an argument when creating the robot object.
The roslaunch command assumes you have urdf_tutorial installed in your catkin_ws - it should be wherever you have your beginner_tutorials and gazebo demos - I'm not sure which of these files you need specifically. I have both, and this runs fine on my laptop. If it doesn't work, and you know you have urdf_tutorials somewhere on  your latop, run:
    - roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/robot_name.urdf'

I think that should solve the problem accordng to the tutorial. Link for the very helpful tutorial: http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch

Comment (as of April 21 (Keya)) - The only thing I haven't been able to figure out is the orientation of the link if the joint is fixed. Right now the code will ask for an orientation, but it wll not be able to actually change it because I ave comented out that bit of code. The orientation change occurs around the center of the link, not the joint position, and so the change in orientation forces the link to no longer be in contact wth the previous link - uncomment the code to see what I mean. It should be clear in the robot_class.py file - I have added four #'s to demarcate it.
