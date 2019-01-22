#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -- Standard
import open3d
import numpy as np
import sys, os, cv2
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# -- ROS
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix

# -- My lib
sys.path.append(PYTHON_FILE_PATH + "../src_python")
from lib_baxter import MyBaxter
from lib_geo_trans_ros import form_T, quaternion_to_R, toRosPose, pose2T
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos

# -- Message types
# from std_msgs.msg import Int32  # used for indexing the ith robot pose
from sensor_msgs.msg import PointCloud2  # for DEBUG_MODE_FOR_BAXTER
from geometry_msgs.msg import Pose, Point, Quaternion
from scan3d_by_baxter.msg import T4x4


# ------------------------------------------------------------

# -- Functions


def moveBaxterToJointAngles(joint_angles):
    if DEBUG_MODE_FOR_BAXTER:
        None
    else:
        my_Baxter.moveToJointAngles(joint_angles)
    return


def readKinectCameraPose():
    if DEBUG_MODE_FOR_BAXTER:  # Manually define the T4x4 matrix
        pos = Point(0, 0, 0)
        quaternion = quaternion_from_euler(0, 0, 0, 'rxyz')
        T = pose2T(pos, quaternion)
    else:
        # (pos, quaternion) = my_Baxter.getFramePose('/left_hand_camera')
        # (pos, quaternion) = my_Baxter.getFramePose('/left_gripper')
        (pos, quaternion) = my_Baxter.getFramePose('/left_lower_forearm')
        T_base_to_arm = pose2T(pos, quaternion)
        T_arm_to_depth  # This is read from file
        T = T_base_to_arm.dot(T_arm_to_depth)
    return T


# int2str with specified width filled by 0
def int2str(x, width): return ("{:0"+str(width)+"d}").format(x)


def savePoseToFile(pose, ith_goalpose):
    filename = file_folder+file_name_pose + int2str(ith_goalpose, 2)+".txt"
    p = pose.position
    q = pose.orientation
    data = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    np.savetxt(filename, data, delimiter=" ")


def getCloudSize(open3d_cloud):
    return np.asarray(open3d_cloud.points).shape[0]


# -- Main
if __name__ == "__main__":
    rospy.init_node('node1')
    DEBUG_MODE_FOR_BAXTER = rospy.get_param("DEBUG_MODE_FOR_BAXTER")
    DEBUG_MODE_FOR_RGBDCAM = rospy.get_param("DEBUG_MODE_FOR_RGBDCAM")

    file_folder = rospy.get_param("file_folder")
    file_name_pose = rospy.get_param("file_name_pose")
    file_name_index_width = rospy.get_param("file_name_index_width")

    config_folder = rospy.get_param("file_folder_config")
    file_name_T_arm_to_depth = rospy.get_param("file_name_T_arm_to_depth")
    T_arm_to_depth = np.loadtxt(config_folder+file_name_T_arm_to_depth)
    # T_baxter_to_chess = np.loadtxt(config_folder+"T_baxter_to_chess.txt")

    # Set Baxter
    if not DEBUG_MODE_FOR_BAXTER:
        my_Baxter = MyBaxter(['left', 'right'][0])
        my_Baxter.enableBaxter()

    # Start node when pressing enter
    rospy.loginfo("\n\nWaiting for pressing 'enter' to start ...")
    raw_input("")

    # Param settings
    topic_endeffector_pos = rospy.get_param("topic_n1_to_n2")
    num_goalposes = rospy.get_param("num_goalposes")

    # Set publisher: After Baxter moves to the next goalpose position,
    #   sends the pose to node2 to tell it to take the picture.
    def publishPose(pose):
        T = pose

        # Trans to 1x16 array
        pose_1x16 = []
        for i in range(4):
            for j in range(4):
                pose_1x16 += [T[i, j]]
        pub_pose.publish(pose_1x16)
        return
    pub_pose = rospy.Publisher(topic_endeffector_pos, T4x4, queue_size=10)

    # Speicify the goalpose_joint_angles that the Baxter needs to move to
    # 7 numbers of the 7 joint angles
    goalposes_joint_angles = [
        [-0.1375, -0.775, -1.4375, 1.65, 1.0, 1.5875, 2.9875],
        [-0.0825, -0.465, -0.8625, 0.99, 0.6, 0.9525, 1.7925],
        [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]
        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]  # its len >= num_goalposes
    num_joints = 7

    # Settings for debug kinect:
    #   Simulate fake point cloud publisher by reading point_cloud from file.
    if DEBUG_MODE_FOR_RGBDCAM:
        debug_file_folder = rospy.get_param("debug_file_folder")
        debug_file_name = rospy.get_param(
            "debug_file_name")  # without suffix "i.pcd"
        topic_name_rgbd_cloud = rospy.get_param("topic_name_rgbd_cloud")

        pub_sim_cloud = rospy.Publisher(topic_name_rgbd_cloud,
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
    while ith_goalpose < num_goalposes and not rospy.is_shutdown():
        ith_goalpose += 1
        joint_angles = goalposes_joint_angles[ith_goalpose-1]

        # Move robot to the next pose for taking picture
        rospy.loginfo("--------------------------------")
        rospy.loginfo("node1: Baxter is moving to pos: "+str(joint_angles))
        moveBaxterToJointAngles(joint_angles)
        rospy.loginfo("node1: Baxter reached the pose!")

        # Publish the signal to node2
        rospy.loginfo("node1: Wait until stable for 1 more second")
        rospy.sleep(1.0)
        rospy.loginfo("node1: publish "+str(ith_goalpose) +
                      "th camera pose to node2")
        pose = readKinectCameraPose()
        publishPose(pose)
        savePoseToFile(pose, ith_goalpose)

        if DEBUG_MODE_FOR_RGBDCAM:  # simulate publishing a point cloud
            rospy.sleep(0.01)
            sim_pub_point_cloud(ith_goalpose)
        rospy.loginfo("--------------------------------")

        rospy.sleep(5)
        # if ith_goalpose==num_goalposes: ith_goalpose = 0

    # -- Node stops
    rospy.loginfo("!!!!! Node 1 stops.")
