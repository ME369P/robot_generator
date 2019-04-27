#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
global Q, nJoints

def callback(key_vel):
    global Q, nJoints
    ## Initiate publishers <FROM PARSER>
    pub1 = rospy.Publisher('rrbot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('rrbot/joint2_position_controller/command', Float64, queue_size=10)

    ## Set robot data
    L_in = [1, 1] # link lengths <FROM PARSER>
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
        V = np.array([-X, -Y])
        qdot = np.linalg.pinv(J).dot(V)
        q_new = Q + qdot*dt

    ## Publish desired joint positions <FROM PARSER>
    pub1.publish(float(q_new[0]))
    pub2.publish(float(q_new[1]))
    Q = q_new

def RRbot_control():
    global Q, nJoints
    nJoints = 2 # number of joints <FROM PARSER>

    ## Initiate a node named RRbot_control
    rospy.init_node('RRbot_control', anonymous=True)
    
    ## Acquire current joint pose
    Jointsdata = rospy.wait_for_message('rrbot/joint_states', JointState)
    Q = np.array([])
    for i in range(nJoints):
        Q = np.append(Q,[Jointsdata.position[i]])

    ## Initiate a subscriber to the key_teleop package
    key_sub = rospy.Subscriber('key_vel', Twist, callback)

    ## Let it spin
    rospy.spin()

    
if __name__ == '__main__':
    try:
        RRbot_control()
    except rospy.ROSInterruptException:
        pass
