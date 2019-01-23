#!/usr/bin/env python
# -*- coding: utf-8 -*-

# rosrun this !

# $ pip install numpy-quaternion
# import quaternion

import open3d
import numpy as np
import sys, os
import cv2
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix

'''
Input and output of relavant functions:
euler_matrix: ea.x, ea.y, ea.z
[q.x, q.y, q.z, q.w] = quaternion_from_euler: [ea.x, ea.y, ea.z]
[ea.x, ea.y, ea.z] = euler_from_quaternion: [q.x, q.y, q.z, q.w]
'''


# Test euler: Result: euler_matrix takes in order of zyx
ea_xyz=[0.3,0.5,1]
R=euler_matrix(ea_xyz[0],ea_xyz[1],ea_xyz[2],'rxyz')[0:3,0:3]
print "\nEuler-angles:",ea_xyz
print "result:",R

# Test Quaternion: result: z, y, x, w
def quaternion_to_matrix(quat_xyzw):
    if type(quat_xyzw)==np.ndarray:
        euler_xyz=euler_from_quaternion(quat_xyzw)
    else: # geometry_msgs.msg.Quaternion
        euler_xyz=euler_from_quaternion([quat_xyzw.x, quat_xyzw.y, quat_xyzw.z, quat_xyzw.w])
    R=euler_matrix(euler_xyz[0],euler_xyz[1],euler_xyz[2],'rxyz')[0:3,0:3]
    return R

ea_xyz=[0,0,0]
quaternion_xyzw = quaternion_from_euler(ea_xyz[0],ea_xyz[1],ea_xyz[2])
print "\n\nxyz-euler:", ea_xyz
print "quaternion_xyzw from the euler:", quaternion_xyzw
print quaternion_to_matrix(quaternion_xyzw)

ea_xyz=[0.3,0.5,1]
quaternion_xyzw = quaternion_from_euler(ea_xyz[0],ea_xyz[1],ea_xyz[2])
print "\n\nxyz-euler:", ea_xyz
print "quaternion_xyzw from the euler:", quaternion_xyzw
quat=Quaternion(quaternion_xyzw[0],quaternion_xyzw[1],quaternion_xyzw[2],quaternion_xyzw[3])
print quaternion_to_matrix(quat)

print "Extra test1:"
def list_to_quat(l):
    quat=Quaternion(l[0],l[1],l[2],l[3])
    return quat
quat = list_to_quat(
    [-0.7806487738947058, 0.6246318499639127, 0.009172542080295257, 0.018395877232085456]
)
print quaternion_to_matrix(quat)

print "Extra test2:"
quat = list_to_quat(
    [-0.7806487738947058, -0.6246318499639127, -0.009172542080295257, 0.018395877232085456]
)
print quaternion_to_matrix(quat)

print "Extra test3:"
quat = list_to_quat(
    [0.7806537138701537, -0.6246224733741084, -0.00940960124324048, 0.018384890023907856]
)
print quaternion_to_matrix(quat)

