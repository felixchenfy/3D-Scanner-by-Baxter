#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tf.transformations import*
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import open3d
import numpy as np
import sys
import os
import cv2
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

"""
ROS tf.transformations param meaning of 'sxyz':

A triple of Euler angles can be applied/interpreted in 24 ways, which can
be specified using a 4 character string or encoded 4-tuple:

  *Axes 4-string*: e.g. 'sxyz' or 'ryxy'

  - first character : rotations are applied to 's'tatic or 'r'otating frame
  - remaining characters : successive rotation axis 'x', 'y', or 'z'

-- My understanding
rxyz: Rx*Ry*Rz
sxyz: what the hell is this???????????????????????????? It's not any of the 6 combinations of Rx Ry Rz
"""

# ------------------------------------------------------------------

# alpha, beta, gamma = 0.123, -1.234, 2.345
# alpha, beta, gamma = 0.0, 0.0, 1.0
# alpha, beta, gamma = 0.0, 1.0, 0.0
# alpha, beta, gamma = 1.0, 0.0, 0.0
# alpha, beta, gamma = 1.0, 1.0, 0.0
alpha, beta, gamma = 0.3, 0.5, 1.0
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
xyz_euler = [alpha, beta, gamma]

# ---------------- Ground truth ---------------- 
print "\n\n-------------------------------------------------------"
I = identity_matrix()
Rx = rotation_matrix(alpha, xaxis)
Ry = rotation_matrix(beta, yaxis)
Rz = rotation_matrix(gamma, zaxis)
R = concatenate_matrices(Rx, Ry, Rz)
euler = euler_from_matrix(R, 'rxyz')

print "\nRx", Rx
print "\nRy", Ry
print "\nRz", Rz
print "\nR = concatenate_matrices(Rx, Ry, Rz):\n", R
print "\nR = Rx.dot(Ry).dot(Rz):\n", Rx.dot(Ry).dot(Rz)
print "\neuler_from_matrix(R, 'rxyz')\n", euler

# Ground truth for [0.3, 0.5, 1.0]
# R = Rx.dot(Ry).dot(Rz):
# [[ 0.47415988 -0.73846026  0.47942554]
#  [ 0.88043793  0.39695095 -0.25934338]
#  [ 0.00120636  0.54507466  0.83838664]]


# ---------------- ROS ---------------- 

def test_euler_matrix(xyz_euler, i, j, k):
    return euler_matrix(xyz_euler[i],xyz_euler[j],xyz_euler[k], 'rxyz')[0:3,0:3]
    # return euler_matrix(xyz_euler[i],xyz_euler[j],xyz_euler[k], 'sxyz')[0:3,0:3]
print "\neuler_matrix(x, y, z)[0:3,0:3]\n", test_euler_matrix(xyz_euler, 0, 1, 2)
# print "\neuler_matrix(x, z, y)[0:3,0:3]\n", test_euler_matrix(xyz_euler, 0, 2, 1) # wrong
# print "\neuler_matrix(y, x, z)[0:3,0:3]\n", test_euler_matrix(xyz_euler, 1, 0, 2) # wrong
# print "\neuler_matrix(y, z, x)[0:3,0:3]\n", test_euler_matrix(xyz_euler, 1, 2, 0) # wrong
# print "\neuler_matrix(z, x, y)[0:3,0:3]\n", test_euler_matrix(xyz_euler, 2, 0, 1) # wrong
# print "\neuler_matrix(z, y, x)[0:3,0:3]\n", test_euler_matrix(xyz_euler, 2, 1, 0) # wrong

# # ---------------- cv2 ---------------- 
def dotdot(R1,R2,R3):
    return R1.dot(R2).dot(R3)
print "\n\n-------------------------------------------------------"
R_vec=np.array([alpha, beta, gamma])
R, _ = cv2.Rodrigues(R_vec)
print "\ncv2.Rodrigues(R_vec)\n", R
# It prints this. What the hell is this?
# [[ 0.4417478  -0.72417111  0.52956122]
#  [ 0.85815164  0.51320408 -0.01404753]
#  [-0.26160016  0.46064929  0.8481554 ]]


# print "\nR = a.dot(b).dot(c):\n", dotdot(Rx, Ry, Rz) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(Rx, Rz, Ry) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(Ry, Rx, Rz) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(Ry, Rz, Rx) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(Rz, Rx, Ry) # Correct
# print "\nR = a.dot(b).dot(c):\n", dotdot(Rz, Ry, Rx) # Wrong

# print "\nR = a.dot(b).dot(c):\n", dotdot(-Rx, -Ry, -Rz) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(-Rx, -Rz, -Ry) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(-Ry, -Rx, -Rz) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(-Ry, -Rz, -Rx) # Wrong
# print "\nR = a.dot(b).dot(c):\n", dotdot(-Rz, -Rx, -Ry) # Correct
# print "\nR = a.dot(b).dot(c):\n", dotdot(-Rz, -Ry, -Rx) # Wrong