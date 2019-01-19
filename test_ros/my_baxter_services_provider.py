#!/usr/bin/python

# Some simple services for printing Baxter status.
''' Command line usage:
    rosservice list |grep my
    rosservice call my/PrintBaxterGripperPose
'''
# Standard
import numpy as np
import sys, os
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# ROS
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

# My
sys.path.append(PYTHON_FILE_PATH + "../src_python")
from lib_baxter import MyBaxter, callService
from scan3d_by_baxter.srv import PrintBaxterGripperPose, PrintBaxterJointAngles
from scan3d_by_baxter.srv import * # import the above's responses types

DEBUG = False
NUM_DIGITS_TO_PRINT = 4

def DEBUG_PROGRAM():
    if DEBUG:
        res=callService("my/PrintBaxterGripperPose", PrintBaxterGripperPose)
        res=callService("my/PrintBaxterJointAngles", PrintBaxterJointAngles)

def list2str(vals):
    str_format="{:."+str(NUM_DIGITS_TO_PRINT)+"f} " #{:.4f}
    str_val = [str_format.format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val

def callback_PrintBaxterGripperPose(req):
    position, quaternion = myBaxter.getGripperPose(flag_return_euler=False)
    position, euler = myBaxter.getGripperPose(flag_return_euler=True)
    # -- Print
    rospy.loginfo("\nPrint Baxter gripper pose:")
    rospy.loginfo("-- x,y,z = " + list2str(position))
    rospy.loginfo("-- quaternion xyzw = " + list2str(quaternion))
    rospy.loginfo("-- euler xyz = " + list2str(euler))
    # -- Output
    # pose = Pose(Point(position[0],position[1],position[2]),
    #     Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
    # return PrintBaxterGripperPoseResponse(True, pose)
    return PrintBaxterGripperPoseResponse()

def callback_PrintBaxterJointAngles(req):
    angles = myBaxter.getJointAngles()
    rospy.loginfo("\nPrint Baxter joint angles:")
    rospy.loginfo("   " + list2str(angles))
    # return PrintBaxterJointAnglesResponse(True, angles)
    return PrintBaxterJointAnglesResponse()
    

if __name__ == "__main__":
    rospy.init_node('my_baxter_services_provider')

    # Setting Baxter
    arm_name = ['left', 'right'][0]
    myBaxter = MyBaxter(arm_name)

    # Services
    s1 = rospy.Service('my/PrintBaxterGripperPose', PrintBaxterGripperPose, callback_PrintBaxterGripperPose)
    s2 = rospy.Service('my/PrintBaxterJointAngles', PrintBaxterJointAngles, callback_PrintBaxterJointAngles)

    # Spin
    DEBUG_PROGRAM()
    rospy.spin()
