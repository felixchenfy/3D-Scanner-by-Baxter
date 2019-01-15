#!/usr/bin/env python
# -*- coding: utf-8 -*-

import open3d
import numpy as np
import sys, os
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

import rospy
from std_msgs.msg import Int32  # used for indexing the ith robot pose
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
    rospy.init_node('work_flow_controller')

    # Param settings
    topic_name_endeffector_pos = "my/endeffector_pos"
    topic_name_take_picture = "my/take_picture"

    # Set publisher
    pub_pose = rospy.Publisher(topic_name_endeffector_pos,
                               Pose, queue_size=10)
    pub_event = rospy.Publisher(topic_name_take_picture,
                                Int32, queue_size=10)

    def sendTakePictureSignal(ith_pos):
        pub_event.publish(Int32(ith_pos))

    # Boot up Baxter
    if not DEBUG_MODE:
        None

    # Speicify the goal_joint_angles that the Baxter needs to move to
    # 7 numbers of the 7 joint angles
    goals_joint_angles = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    num_goals = len(goals_joint_angles)
    num_joints = 7

    # Settings for DEBUG_MODE
    if DEBUG_MODE:
        # Read point_cloud file for simulating fake point cloud publisher
        cloud_folder = "../data_debug/"
        cloud_filename = "cloud_cluster_"
        topic_name_kinect_cloud = "kinect2/qhd/points"
        pub_sim_cloud = rospy.Publisher(topic_name_kinect_cloud,
            PointCloud2, queue_size=10)
        def sim_pub_point_cloud(ith_pos):
            filename = cloud_filename+str(ith_pos)+".pcd"
            filename_whole = PYTHON_FILE_PATH+cloud_folder+filename
            open3d_cloud = open3d.read_point_cloud(filename_whole)
            rospy.loginfo("node1: sim: load cloud file: " + filename +
                ", points = " + str(getCloudSize(open3d_cloud)))
            ros_cloud = convertCloudFromOpen3dToRos(open3d_cloud)
            rospy.loginfo("node1: sim: publishing cloud "+str(ith_pos))
            pub_sim_cloud.publish(ros_cloud)

    # Robot moves to position
    rospy.sleep(1)
    for ith_pos, joint_angles in enumerate(goals_joint_angles):
        moveBaxterToJointAngles(joint_angles)
        pose = readBaxterEndeffectPose()
        
        rospy.loginfo("--------------------------------")
        rospy.loginfo("node1: publish pose "+str(ith_pos))
        pub_pose.publish(pose)

        rospy.loginfo("node1: publish signal to take picture " + str(ith_pos))
        sendTakePictureSignal(ith_pos)
        
        if DEBUG_MODE:
            rospy.sleep(0.01)
            sim_pub_point_cloud(ith_pos)

        rospy.sleep(2)
