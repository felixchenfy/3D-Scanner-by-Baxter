#!/usr/bin/env python
# -*- coding: utf-8 -*-

import open3d
import numpy as np
import sys, os
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

import rospy
# from std_msgs.msg import Int32  # used for indexing the ith robot pose
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2 # for DEBUG_MODE

# Include my lib
sys.path.append(PYTHON_FILE_PATH + "../src_ros")
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos

# ------------------------------------------------------------
# -- Param settings
DEBUG_MODE = True

# -- Functions
def moveBaxterToJointAngles(pos):
    if DEBUG_MODE:
        None
    else:
        None  # read


def readBaxterEndeffectPose():
    if DEBUG_MODE:
        pose = Pose()
    else:
        position = Point()
        orientation = Quaternion()
        pose = Pose(position, orientation)
    return pose

def getCloudSize(open3d_cloud):
    return np.asarray(open3d_cloud.points).shape[0]
    
# -- Main
if __name__ == "__main__":
    rospy.init_node('node1')

    # Param settings
    topic_endeffector_pos = rospy.get_param("topic_n1_to_n2")

    # Set publisher: After Baxter moves to the next goalpose position, 
    #   sends the pose to node2 to tell it to take the picture.
    pub_pose = rospy.Publisher(topic_endeffector_pos,
                               Pose, queue_size=10)

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
    num_goalposes = len(goalposes_joint_angles)
    num_joints = 7

    # Settings for DEBUG_MODE
    if DEBUG_MODE:
        # Read point_cloud file for simulating fake point cloud publisher

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
    while ith_goalpose<num_goalposes:
        joint_angles=goalposes_joint_angles[ith_goalpose]

        # Move robot, and then take picture
        moveBaxterToJointAngles(joint_angles)
        pose = readBaxterEndeffectPose()
        
        rospy.loginfo("--------------------------------")
        rospy.loginfo("node1: publish pose "+str(ith_goalpose))
        pub_pose.publish(pose)
        
        if DEBUG_MODE: # simulate publishing a point cloud
            rospy.sleep(0.01)
            sim_pub_point_cloud(ith_goalpose)

        rospy.sleep(2)
        ith_goalpose+=1
        if ith_goalpose==num_goalposes: ith_goalpose = 0
