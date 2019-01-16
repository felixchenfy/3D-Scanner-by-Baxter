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


alpha, beta, gamma = 0.3, 0.3, 1.0
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
I = identity_matrix()
Rx = rotation_matrix(alpha, xaxis)
Ry = rotation_matrix(beta, yaxis)
Rz = rotation_matrix(gamma, zaxis)
R = concatenate_matrices(Rx, Ry, Rz)
euler = euler_from_matrix(R, 'rxyz')
numpy.allclose([alpha, beta, gamma], euler)

Re = euler_matrix(alpha, beta, gamma, 'rxyz')
# Re = euler_matrix(alpha, beta, gamma)
is_same_transform(R, Re)
print "\n\n euler:", euler
print "\n\n euler_matrix ground truth:\n", R
print "\n\n euler_matrix euler_matrix:\n", Re

al, be, ga = euler_from_matrix(Re, 'rxyz')
is_same_transform(Re, euler_matrix(al, be, ga, 'rxyz'))

qx = quaternion_about_axis(alpha, xaxis)
qy = quaternion_about_axis(beta, yaxis)
qz = quaternion_about_axis(gamma, zaxis)
q = quaternion_multiply(qx, qy)
q = quaternion_multiply(q, qz)
Rq = quaternion_matrix(q)
is_same_transform(R, Rq)

S = scale_matrix(1.23, origin)
T = translation_matrix((1, 2, 3))
Z = shear_matrix(beta, xaxis, origin, zaxis)
R = random_rotation_matrix(numpy.random.rand(3))
M = concatenate_matrices(T, R, Z, S)
scale, shear, angles, trans, persp = decompose_matrix(M)
numpy.allclose(scale, 1.23)

numpy.allclose(trans, (1, 2, 3))

numpy.allclose(shear, (0, math.tan(beta), 0))

is_same_transform(R, euler_matrix(axes='sxyz', *angles))

M1 = compose_matrix(scale, shear, angles, trans, persp)
is_same_transform(M, M1)
