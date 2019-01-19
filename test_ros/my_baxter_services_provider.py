#!/usr/bin/python

# Some simple services for printing Baxter status.
''' Command line usage:
    rosservice list |grep my
    rosservice call my/PrintBaxterGripperPose
    rosservice call my/PrintBaxterJointAngles
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
NUM_DECIMALS_TO_PRINT = 4 

def DEBUG_PROGRAM():
    if DEBUG:
        res=callService("my/PrintBaxterGripperPose", PrintBaxterGripperPose)
        res=callService("my/PrintBaxterJointAngles", PrintBaxterJointAngles)

def list2str(vals):
    str_format="{:."+str(NUM_DECIMALS_TO_PRINT)+"f} " #{:.4f}
    str_val = [str_format.format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val

def truncateFloat(val, num_decimals = 4): # (0.123456789, 4) -> 0.1234
    return val
    # -- These function is commented out.
    # -- The reason is, the float number to ros service becomes inaccurate. This func will 
    # -- return something like this: -1.7077000141143799, even if I set it 4 decimals. 
    # ratio_ = 10**num_decimals
    # truncate_ = lambda x: round(x*ratio_)*1.0/ratio_
    # if type(val)==list:
    #     return [truncate_(x) for x in val]
    # else:
    #     return truncate_(val)

def callback_PrintBaxterGripperPose(req):
    position, quaternion = myBaxter.getGripperPose(flag_return_euler=False)
    position, euler = myBaxter.getGripperPose(flag_return_euler=True)
    if DEBUG:
        rospy.loginfo("")
        rospy.loginfo("Print Baxter gripper pose:")
        rospy.loginfo("-- x,y,z = " + list2str(position))
        rospy.loginfo("-- quaternion xyzw = " + list2str(quaternion))
        rospy.loginfo("-- euler xyz = " + list2str(euler))
    return PrintBaxterGripperPoseResponse(True,
        truncateFloat(position), truncateFloat(quaternion), truncateFloat(euler))

def callback_PrintBaxterJointAngles(req):
    angles = myBaxter.getJointAngles()
    if DEBUG:
        rospy.loginfo("")
        rospy.loginfo("Print Baxter joint angles:")
        rospy.loginfo("   " + list2str(angles))
    return PrintBaxterJointAnglesResponse(True, truncateFloat(angles))
    

if __name__ == "__main__":
    rospy.init_node('my_baxter_services_provider')

    # Setting Baxter
    arm_name = ['left', 'right'][0]
    myBaxter = MyBaxter(arm_name)

    # Services
    s1 = rospy.Service('my/PrintBaxterGripperPose', PrintBaxterGripperPose, callback_PrintBaxterGripperPose)
    s2 = rospy.Service('my/PrintBaxterJointAngles', PrintBaxterJointAngles, callback_PrintBaxterJointAngles)

    # Spin
    rospy.loginfo("\n\nmy_baxter_services_provider starts!\n")
    DEBUG_PROGRAM()
    rospy.spin()
