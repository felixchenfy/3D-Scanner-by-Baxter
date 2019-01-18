#!/usr/bin/env python
# -*- coding: utf-8 -*-

import open3d
import numpy as np
import sys, os
import cv2
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

import rospy
# from std_msgs.msg import Int32  # used for indexing the ith robot pose
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2 # for DEBUG_MODE
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix

# Include my lib
sys.path.append(PYTHON_FILE_PATH + "../src_python")
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos
from lib_geo_trans import form_T, quaternion_to_SO3
from scan3d_by_baxter.msg import T4x4

# ------------------------------------------------------------
# -- Param settings
DEBUG_MODE = True

# -- Functions
def moveBaxterToJointAngles(pos):
    if DEBUG_MODE:
        None
    else:
        (trans, rot) = self.tf_listener.lookupTransform(
            '/base', '/left_hand_camera', rospy.Time(0))
        # print "quaternion from tf = ", rot
        r3=euler_from_quaternion(rot)
        # print "euler_matrix=",euler_matrix(r3[0],r3[1],r3[2])
        R=euler_matrix(r3[0],r3[1],r3[2])[0:3,0:3]
        # R,_=cv2.Rodrigues(r3)
        # R=np.linalg.inv(R)
        T = form_T(R,trans)
        return T

def readBaxterEndeffectPose():
    if DEBUG_MODE: # Manually define the T4x4 matrix
        pos = Point(-1, 1, 0)
        pos = Point(0, 0, 0)
        quaternion = quaternion_from_euler(0, 0, 0, 'rxyz')
    else:
        (pos, quaternion) = self.tf_listener.lookupTransform(
            '/base', '/left_hand_camera', rospy.Time(0))
    return Pose(pos, quaternion)

def getCloudSize(open3d_cloud):
    return np.asarray(open3d_cloud.points).shape[0]
    
# -- Main
if __name__ == "__main__":
    rospy.init_node('node1')
    print("Waiting for keypress to start ...")
    cv2.waitKey(0)

    # Param settings
    topic_endeffector_pos = rospy.get_param("topic_n1_to_n2")
    num_goalposes = rospy.get_param("num_goalposes")

    # Set publisher: After Baxter moves to the next goalpose position, 
    #   sends the pose to node2 to tell it to take the picture.
    pub_pose = rospy.Publisher(topic_endeffector_pos,
                               T4x4, queue_size=10)
    def publishPose(pose):
        # Trans to 4x4 matrix
        R = quaternion_to_SO3(pose.orientation) # this is my func. support both list and quat-struct.
        p = [pose.position.x, pose.position.y, pose.position.z]
        T = form_T(R, p)

        # Trans to 1x16 array
        pose_1x16 = []
        for i in range(4):
            for j in range(4):
                pose_1x16+=[T[i,j]]
        pub_pose.publish(pose_1x16)

    # Boot up Baxter
    if not DEBUG_MODE:
        None

    # Speicify the goalpose_joint_angles that the Baxter needs to move to
    # 7 numbers of the 7 joint angles
    goalposes_joint_angles = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    num_joints = 7

    # Settings for DEBUG_MODE:
    #   Read point_cloud file for simulating fake point cloud publisher
    if DEBUG_MODE:
        debug_file_folder = rospy.get_param("debug_file_folder")
        debug_file_name = rospy.get_param("debug_file_name") # without suffix "i.pcd"
        topic_name_kinect_cloud = rospy.get_param("topic_name_kinect_cloud")

        pub_sim_cloud = rospy.Publisher(topic_name_kinect_cloud,
            PointCloud2, queue_size=10)
        def sim_pub_point_cloud(ith_goalpose):
            filename = debug_file_name+str(ith_goalpose)+".pcd"
            filename_whole = debug_file_folder+filename
            open3d_cloud = open3d.read_point_cloud(filename_whole)
            rospy.loginfo("node1: sim: load cloud file: " + filename +
                ", points = " + str(getCloudSize(open3d_cloud)))
            ros_cloud = convertCloudFromOpen3dToRos(open3d_cloud)
            rospy.loginfo("node1: sim: publishing cloud "+str(ith_goalpose))
            pub_sim_cloud.publish(ros_cloud)

    # Robot moves to position
    rospy.sleep(1)
    # for ith_goalpose, joint_angles in enumerate(goalposes_joint_angles):
    ith_goalpose = 0 
    while ith_goalpose<num_goalposes and not rospy.is_shutdown():
        joint_angles=goalposes_joint_angles[ith_goalpose]

        # Move robot, and then take picture
        moveBaxterToJointAngles(joint_angles)
        pose = readBaxterEndeffectPose()
        
        rospy.loginfo("--------------------------------")
        rospy.loginfo("node1: publish pose "+str(ith_goalpose))
        publishPose(pose)

        if DEBUG_MODE: # simulate publishing a point cloud
            rospy.sleep(0.01)
            sim_pub_point_cloud(ith_goalpose)
        rospy.loginfo("--------------------------------")

        rospy.sleep(2)
        ith_goalpose+=1
        # if ith_goalpose==num_goalposes: ith_goalpose = 0

    # -- Node stops
    rospy.loginfo("!!!!! Node 1 stops.")
