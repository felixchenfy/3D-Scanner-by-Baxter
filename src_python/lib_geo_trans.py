import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import euler_matrix, euler_from_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Point, Quaternion
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

# a bit wrap for geometry_msgs.msg.Pose
def toRosPose(pos, quaternion):
    if(type(pos)==list):
        pos = Point(pos[0],pos[1],pos[2])
    if(type(quaternion)==list):
        quaternion = Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    return Pose(pos, quaternion)

def quaternion_to_R(quat_xyzw):
    if type(quat_xyzw) != list:
        quat_xyzw=[quat_xyzw.x, quat_xyzw.y, quat_xyzw.z, quat_xyzw.w]
    euler_xyz = euler_from_quaternion(quat_xyzw)
    R = euler_matrix(euler_xyz[0], euler_xyz[1],
                     euler_xyz[2], 'rxyz')[0:3, 0:3]
    return R

def Rp_to_pose(R, p):
    if R.shape[0]==3: # expand the matrix to 4x4
        tmp=np.identity(4)
        tmp[0:3,0:3]=R
        R=tmp
    quaternion = quaternion_from_matrix(R)
    return Pose(p ,quaternion)



if __name__=="__main__":

    # -- Prepare data
    p = [1,2,3]
    euler = [ 0.3, 0.5, 1.0]
    R = euler_matrix(euler[0],euler[1],euler[2], 'rxyz')[0:3,0:3]
    quaternion = quaternion_from_euler(
        euler[0],euler[1],euler[2]) # [0.24434723 0.1452622  0.4917509  0.82302756]
    
    # -----------------------------------------------------------------------
    # -- Copy test case below

    # ================================
    # print(form_T(R, p))

    # ================================
    # print(R)
    # print(quaternion_to_R(quaternion))

    # ================================
    # print(quaternion)
    # print(toRosPose(p, quaternion))

    # ================================
    # print(Rp_to_pose(R, p))
