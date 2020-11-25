#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

# joints[0], z axis, 2.5m, yellow
# joints[1], x axis, 0m, blue
# joints[2], y axis, 3.5m, blue
# joints[3], x axis, 3m, green
def ForwardKinematics(joints):
    # simplified with WolframAlpha
    x = 3*np.sin(joints[0])*np.sin(joints[1])*np.cos(joints[2])*np.cos(joints[3]) + 3.5*np.sin(joints[0])*np.sin(joints[1])*np.cos(joints[2]) + 3*np.sin(joints[0])*np.cos(joints[1])*np.sin(joints[3])+ 3*np.cos(joints[0])*np.sin(joints[2])*np.cos(joints[3]) + 3.5*np.cos(joints[0])*np.sin(joints[2])
    y = -3*np.cos(joints[0])*np.sin(joints[1])*np.cos(joints[2])*np.cos(joints[3]) - 3.5*np.cos(joints[0])*np.sin(joints[1])*np.cos(joints[2]) - 3*np.cos(joints[0])*np.cos(joints[1])*np.sin(joints[3]) + 3*np.sin(joints[0])*np.sin(joints[2])*np.cos(joints[3]) + 3.5*np.sin(joints[0])*np.sin(joints[2])
    z = 3*np.cos(joints[1])*np.cos(joints[2])*np.cos(joints[3]) + 3.5*np.cos(joints[1])*np.cos(joints[2]) - 3*np.sin(joints[1])*np.sin(joints[3]) + 2.5
    EndEffector = np.array([x,y,z])
    return EndEffector

# def ClosedLoop():

def JacobianMatrix(joints):
    # calculated with WolframAlpha
    x0 = np.cos(joints[0])*(np.sin(joints[1])*np.cos(joints[2])*(3*np.cos(joints[3]) + 3.5) + 3*np.cos(joints[1])*np.sin(joints[3])) + np.sin(joints[0])*np.sin(joints[2])*(-3*np.cos(joints[3]) - 3.5)
    x1 = np.sin(joints[0])*(np.cos(joints[1])*np.cos(joints[2])*(3*np.cos(joints[3]) + 3.5) - 3*np.sin(joints[1])*np.sin(joints[3]))
    x2 = 0.5*(6*np.cos(joints[3]) + 7)*(np.cos(joints[0])*np.cos(joints[2]) - np.sin(joints[0])*np.sin(joints[1])*np.sin(joints[2]))
    x3 = np.sin(joints[0])*(3*np.cos(joints[1])*np.cos(joints[3]) - 3*np.sin(joints[1])*np.cos(joints[2])*np.sin(joints[3])) - 3*np.cos(joints[0])*np.sin(joints[2])*np.sin(joints[3])
    y0 = np.sin(joints[0])*(np.sin(joints[1])*np.cos(joints[2])*(3*np.cos(joints[3]) + 3.5) + 3*np.cos(joints[1])*np.sin(joints[3])) + np.cos(joints[0])*np.sin(joints[2])*(3*np.cos(joints[3]) + 3.5)
    y1 = np.cos(joints[0])*(np.cos(joints[1])*np.cos(joints[2])*(-3*np.cos(joints[3]) - 3.5) + 3*np.sin(joints[1])*np.sin(joints[3]))
    y2 = 0.5*(6*np.cos(joints[3]) + 7)*(np.cos(joints[0])*np.sin(joints[1])*np.sin(joints[2]) + np.sin(joints[0])*np.cos(joints[2]))
    y3 = -3*(-np.cos(joints[0])*np.sin(joints[1])*np.cos(joints[2])*np.sin(joints[3]) + np.cos(joints[0])*np.cos(joints[1])*np.cos(joints[3]) + np.sin(joints[0])*np.sin(joints[2])*np.sin(joints[3]))
    z0 = 0.0
    z1 = np.sin(joints[1])*np.cos(joints[2])*(-3*np.cos(joints[3]) - 3.5) - 3*np.cos(joints[1])*np.sin(joints[3])
    z2 = np.cos(joints[1])*np.sin(joints[2])*(-3*np.cos(joints[3]) - 3.5)
    z3 = -3*(np.cos(joints[1])*np.cos(joints[2])*np.sin(joints[3]) + np.sin(joints[1])*np.cos(joints[3]))

    jacobian = np.array([[x0,x1,x2,x3],[y0,y1,y2,y3],[z0,z1,z2,z3]])

    return jacobian


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
