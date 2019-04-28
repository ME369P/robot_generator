#!/usr/bin/env python

import rospy
import numpy as np
from os.path import dirname, abspath
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
global Q, nJoints, real_name, L_in

def callback(key_vel):
    global Q, nJoints, real_name, L_in
    ## Initiate publishers
    pub = []
    for i in range(nJoints):
        pub.append(rospy.Publisher('{}/joint{}_position_controller/command'.format(real_name,i+1), Float64, queue_size=10))

    ## Set robot data
    L = np.array([])
    for i in range(nJoints):
        L = np.append(L,L_in[i])

    ## Calculate Jacobian
    J1 = np.array([])
    J2 = np.array([])
    for i in range(nJoints):
        X,Y = 0,0
        for k in range(i,nJoints):
            X = X + L[k]*np.cos(sum(Q[0:k+1]))
            Y = Y + L[k]*np.sin(sum(Q[0:k+1]))
        J1 = np.append(J1,[-Y])
        J2 = np.append(J2,[X])
    J = np.append([J1],[J2],axis=0)

    ## Acquire velocity input from communication UI
    scale = 10
    Vx_desired = key_vel.linear.x*scale
    Vy_desired = -key_vel.angular.z*scale
    V = np.array([Vx_desired, Vy_desired])	

    ## Set timestep
    dt = 0.001

    ## Solve inverse kinematics
    qdot = np.linalg.pinv(J).dot(V)
    q_new = Q + qdot*dt

    ## Check to avoid singularity
    X,Y = 0,0
    for i in range(nJoints):
        X = X + L[i]*np.cos(sum(Q[0:i+1]))
        Y = Y + L[i]*np.sin(sum(Q[0:i+1]))

    if np.hypot(X,Y) > sum(L)-0.01:
        V = np.array([-X,-Y])
        qdot = np.linalg.pinv(J).dot(V)
        if np.sqrt(qdot.dot(qdot)) > 5: qdot = qdot/np.sqrt(qdot.dot(qdot))*5
        q_new = Q + qdot*dt
    
    ## Publish desired joint positions
    Q = q_new
    i = 0
    for pubs in pub:
        pubs.publish(float(q_new[i]))
        i = i + 1
    

def RRbot_control():
    global Q, nJoints, real_name, L_in
    
    try:
        robot_name = str(input('Enter the name of the robot:'))
        
        line_list = []
    
        abs_path = dirname(abspath(__file__))
        with open("{}/{}_info.txt".format(abs_path,robot_name)) as f:
            for line in f:
                line_list.append(line.rstrip('\n'))
                
            real_name = line_list[0]
            L = line_list[1][4:-1].split(', ')
            nJoints = len(L)
            L_in = []
            for i in range(nJoints):
                L_in.append(float(L[i]))
            
        ## Initiate a node named RRbot_control
        rospy.init_node('{}_control'.format(robot_name), anonymous=True)
        ## Acquire current joint pose
        Jointsdata = rospy.wait_for_message('{}/joint_states'.format(real_name), JointState)
        Q = np.array([])
        for i in range(nJoints):
            Q = np.append(Q,[Jointsdata.position[i]])
    
        ## Initiate a subscriber to the key_teleop package
        key_sub = rospy.Subscriber('key_vel', Twist, callback)
        
        ## Let it spin
        rospy.spin()
    except NameError:
        print("Put ' ' around the robot name.")

if __name__ == '__main__':
    try:
        RRbot_control()
    except rospy.ROSInterruptException:
        pass
