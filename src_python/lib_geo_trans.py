import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
from geometry_msgs.msg import Pose, Point
import cv2


def form_T(R, p):
    T = np.identity(4)
    T[0:3, 0:3] = R
    try:
        T[0:3, 3:4] = p[0:3, 0:1]
    except:
        T[0, 3] = p[0]
        T[1, 3] = p[1]
        T[2, 3] = p[2]
    return T


def get_Rp_from_T(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3:4]
    return (R, p)


def quaternion_to_SO3(quat_xyzw):
    if type(quat_xyzw) == np.ndarray:
        xyz_euler = euler_from_quaternion(quat_xyzw)
    else:  # type == geometry_msgs.msg.Quaternion
        xyz_euler = euler_from_quaternion(
            [quat_xyzw.x, quat_xyzw.y, quat_xyzw.z, quat_xyzw.w])
    R = euler_matrix(xyz_euler[0], xyz_euler[1],
                     xyz_euler[2], 'rxyz')[0:3, 0:3]
    return R

# def Rp_to_pose(R,p):
#     pose=Pose()

#     R_vec, _ = cv2.Rodrigues(R)
#     q=quaternion_from_euler(R_vec[2],R_vec[1],R_vec[0])
#     pose.orientation.w=q[0]
#     pose.orientation.x=q[1]
#     pose.orientation.y=q[2]
#     pose.orientation.z=q[3]

#     pose.position.x=p[0]
#     pose.position.y=p[1]
#     pose.position.z=p[2]
#     return pose
