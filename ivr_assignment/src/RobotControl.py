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

class RobotControl:

    def __init__(self):
        # initialize the node named RobotControl
        rospy.init_node('RobotControl', anonymous=True)

        # initialize publishers
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.ForwardKinematics_pub = rospy.Publisher("/forward_kinematics_result", Float64MultiArray, queue_size=10)

        self.joints = np.array([0.0,0.0,0.0,0.0], dtype='float64')
        self.target_pos = []

        # initialize subscribers
        self.target_sub = rospy.Subscriber("/target_pos",Float64MultiArray, self.set_target)
        self.joints_value_sub = rospy.Subscriber("/robot/joints_value",Float64MultiArrayself.set_joints)
        self.end_effector_sub = rospy.Subscriber("/end_pos",Float64MultiArray,self.set_EndEffector)
        self.end_effector_sub2 = rospy.Subscriber("/end_pos",Float64MultiArray,self.callback)

        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')

        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0,0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0,0.0,0.0], dtype='float64')

    def set_target(self, target):
        self.target_pos = np.array(target.data)

    # def set_joints(self, ):

    def set_EndEffector(self, EndEffector):
      self.end_pos = np.array(EndEffector.data)

    def ForwardKinematics(self, joints):
        # joints[0], z axis, 2.5m, yellow
        # joints[1], x axis, 0m, blue
        # joints[2], y axis, 3.5m, blue
        # joints[3], x axis, 3m, green
        # simplified with WolframAlpha
        x = 3*np.sin(joints[0])*np.sin(joints[1])*np.cos(joints[2])*np.cos(joints[3]) + 3.5*np.sin(joints[0])*np.sin(joints[1])*np.cos(joints[2]) + 3*np.sin(joints[0])*np.cos(joints[1])*np.sin(joints[3])+ 3*np.cos(joints[0])*np.sin(joints[2])*np.cos(joints[3]) + 3.5*np.cos(joints[0])*np.sin(joints[2])
        y = -3*np.cos(joints[0])*np.sin(joints[1])*np.cos(joints[2])*np.cos(joints[3]) - 3.5*np.cos(joints[0])*np.sin(joints[1])*np.cos(joints[2]) - 3*np.cos(joints[0])*np.cos(joints[1])*np.sin(joints[3]) + 3*np.sin(joints[0])*np.sin(joints[2])*np.cos(joints[3]) + 3.5*np.sin(joints[0])*np.sin(joints[2])
        z = 3*np.cos(joints[1])*np.cos(joints[2])*np.cos(joints[3]) + 3.5*np.cos(joints[1])*np.cos(joints[2]) - 3*np.sin(joints[1])*np.sin(joints[3]) + 2.5
        EndEffector = np.array([x,y,z])
        return EndEffector

    def JacobianMatrix(self):
        # calculated with WolframAlpha
        joints = self.joints
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

    def ClosedLoopControl(self, joints, EndEffector, target):
        # P gain and D gain
        K_p = np.array([[10,0,0],[0,10,0],[0,0,10]])
        K_d = np.array([[0.5,0,0],[0,0.5,0],[0,0,0.5]])

        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        # robot end-effector position
        pos = EndEffector
        pos_d = target

        # estimate derivative of error and error
        self.error_d = ((pos_d - pos) - self.error)/dt
        self.error = pos_d-pos

        J_inv = np.linalg.pinv(self.JacobianMatrix())  # calculating the psudeo inverse of Jacobian
        dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
        q_d = joints + (dt * dq_d)  # control input (angular position of joints)
        self.joints = q_d
        return q_d

    def callback(self):
        print("Forward Kinematics: ")
        print(self.ForwardKinematics(self.joints))

        q = self.ClosedLoopControl(self.joints, self.end_pos, self.target_pos)

        self.joint1 = Float64()
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.joint1.data = q[0]
        self.joint2.data = q[1]
        self.joint3.data = q[2]
        self.joint4.data = q[3]

        self.robot_joint1_pub.publish(self.joint1)
        self.robot_joint2_pub.publish(self.joint2)
        self.robot_joint3_pub.publish(self.joint3)
        self.robot_joint4_pub.publish(self.joint4)

        rospy.Rate(30).sleep()

def main(args):
    rc = RobotControl()
    try:
    rospy.spin()
    except KeyboardInterrupt:
    print("Shutting down")
    # cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
