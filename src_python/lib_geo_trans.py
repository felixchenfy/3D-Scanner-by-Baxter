
'''
    Geometric and camera related transformations
'''

import numpy as np
import copy
import cv2
from tf.transformations import*

# ---------------- Basic trans


def form_T(R, p):
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3:4] = np.array(p).reshape((3, 1))
    return T


def get_Rp_from_T(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3:4]
    return (R, p)


def invRp(R, p):
    T = form_T(R, p)
    T = np.linalg.inv(T)
    R_inv, p_inv = get_Rp_from_T(T)
    return R_inv, p_inv

def transXYZ(x=None, y=None, z=None):
    T=np.identity(4)
    data=[x, y, z]
    for i in range(3):
        if data[i] is not None:
            T[i, 3]=data[i]
    return T
    
# ---------------- Euler angles


def rot3x3_to_4x4(R):
    T = np.identity(4)
    T[0:3, 0:3] = R
    return T

def rot(axis, angle, matrix_len=3):
    R_vec = np.array(axis).astype(float)*angle
    R, _ = cv2.Rodrigues(R_vec)
    if matrix_len == 4:
        R = rot3x3_to_4x4(R)
    return R

def rotx(angle, matrix_len=3):
    return rot([1,0,0], angle, matrix_len)

def roty(angle, matrix_len=3):
    return rot([0,1,0], angle, matrix_len)

def rotz(angle, matrix_len=3):
    return rot([0,0,1], angle, matrix_len)

def euler2matrix(x, y, z, order='rxyz'):
    return rotx(x).dot(roty(y)).dot(rotz(z))


# ----------------Point's pos transformation between world/camera/image

# Distort a point. Input x,y are the point's pos on the camera normalized plane (z=0)


def distortPoint(x, y, distortion_coeffs):
    r2 = x*x + y*y
    r4 = r2*r2
    r6 = r4*r2
    d = distortion_coeffs
    k1, k2, p1, p2, k3 = d[0], d[1], d[2], d[3], d[4]
    x_distort = x * (1 + k1 * r2 + k2 * r4 + k3 * r6) + \
        2*p1*x*y + p2*(r2 + 2*x*x)
    y_distort = y * (1 + k1 * r2 + k2 * r4 + k3 * r6) + \
        p1*(r2 + 2*y*y) + 2*p2*x*y
    pt_cam_distort = np.array([[x_distort, y_distort, 1]]).transpose()
    return pt_cam_distort

# Represent a point from using world coordinate to using camera coordinate


def world2cam(p, T_cam_to_world):
    if type(p) == list:
        p = np.array(p).reshape((len(p), 1))
    if p.shape[0] == 3:
        p = np.vstack((p, 1))
    p_cam = T_cam_to_world.dot(p)
    return p_cam[0:3, 0:1]

# Project a point represented in camera coordinate onto the image plane


def cam2pixel(p, camera_intrinsics, distortion_coeffs=None):

    # Transform to camera normalized plane (z=1)
    p = p/p[2, 0]  # z=1

    # Distort point
    if distortion_coeffs is not None:
        p = distortPoint(p[0, 0], p[1, 0], distortion_coeffs)

    # Project to image plane
    pt_pixel_distort = camera_intrinsics.dot(p)
    return pt_pixel_distort[0:2, 0]

# A combination of the above two


def world2pixel(p, T_cam_to_world, camera_intrinsics, distortion_coeffs):
    return cam2pixel(world2cam(p, T_cam_to_world), camera_intrinsics, distortion_coeffs)


# ---------------------- Test -----------------------
if __name__ == "__main__":

    R = euler2matrix(np.pi/2, 0, 0)
    # R = rotz(np.pi/2)
    # R = rotx(np.pi/2)
    print R
