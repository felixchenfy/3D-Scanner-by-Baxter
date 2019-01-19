#!/usr/bin/env python
# -*- coding: utf-8 -*-

# http://docs.ros.org/jade/api/tf/html/python/transformations.html


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

function euler_matrix(ea_x, ea_y, ea_z, option){
    option="rxyz": Rx*Ry*Rz
    option="sxyz": what the hell is this?? It's not any of the 6 combinations of Rx Ry Rz
}

Official explanation{
    ROS tf.transformations param meaning of 'sxyz':

    A triple of Euler angles can be applied/interpreted in 24 ways, which can
    be specified using a 4 character string or encoded 4-tuple:

    *Axes 4-string*: e.g. 'sxyz' or 'ryxy'

    - first character : rotations are applied to 's'tatic or 'r'otating frame
    - remaining characters : successive rotation axis 'x', 'y', or 'z'
}


"""

# ------------------------------------------------------------------

alpha, beta, gamma = 0.3, 0.5, 1.0
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
euler_xyz = [alpha, beta, gamma]

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

def test_euler_matrix(euler_xyz, i, j, k):
    return euler_matrix(euler_xyz[i],euler_xyz[j],euler_xyz[k], 'rxyz')[0:3,0:3]
    # return euler_matrix(euler_xyz[i],euler_xyz[j],euler_xyz[k], 'sxyz')[0:3,0:3]
print "\neuler_matrix(x, y, z)[0:3,0:3]\n", test_euler_matrix(euler_xyz, 0, 1, 2)