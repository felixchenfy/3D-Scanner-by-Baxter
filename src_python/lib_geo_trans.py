
'''
    Geometric and camera related transformations
'''

import numpy as np
import copy
import cv2

# ---------------- Basic trans
def form_T(R, p):
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3:4] = np.array(p).reshape((3,1))
    return T

def get_Rp_from_T(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3:4]
    return (R, p)

def invRp(R, p):
    T=form_T(R,p)
    T=np.linalg.inv(T)
    R_inv,p_inv=get_Rp_from_T(T)
    return R_inv, p_inv

# ----------------Point's pos transformation between world/camera/image
def world2cam(p, T_cam_to_world): # to camera normalized plane (z=1)
    if type(p)==list:
        p=np.array(p).reshape((len(p),1))
    if p.shape[0]==3:
        p=np.vstack((p, 1))
    p_cam = T_cam_to_world.dot(p)
    return p_cam[0:3,0:1]/p_cam[2,0] # z=1

def cam2pixel(p, camera_intrinsics, distortion_coeffs = None):
    if distortion_coeffs is not None:
        # If distortion_coeffs is not np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
        #   then we need to undistort points
        x=p[0,0]
        y=p[1,0]
        r2 = x*x + y*y
        r4 = r2*r2
        r6 = r4*r2
        d = distortion_coeffs
        k1,k2,k3,p1,p2=d[0],d[1],d[2],d[3],d[4]
        x_distort = x * (1 + k1 * r2 + k2 * r4 + k3 * r6)+ 2*p1*x*y + p2*(r2 + 2*x*x)
        y_distort = y * (1 + k1 * r2 + k2 * r4 + k3 * r6)+ p1*(r2 + 2*y*y) + 2*p2*x*y
        pt_cam_distort=np.array([[x_distort, y_distort, 1]]).transpose()
        p = pt_cam_distort
        
    pt_pixel_distort=camera_intrinsics.dot(p)
    return pt_pixel_distort[0:2,0]

def world2pixel(p, T_cam_to_world, camera_intrinsics, distortion_coeffs):
    return cam2pixel(world2cam(p, T_cam_to_world), camera_intrinsics, distortion_coeffs)