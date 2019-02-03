#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -- Standard
import open3d
import numpy as np
import sys, os, cv2
from copy import copy
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


def moveBaxterToJointAngles(joint_angles, time_cost = 3.0):
    if DEBUG_MODE_FOR_BAXTER:
        None
    else:
        my_Baxter.moveToJointAngles(joint_angles, time_cost)
    return


def readKinectCameraPose():
    if DEBUG_MODE_FOR_BAXTER:  # Manually define the T4x4 matrix
        pos = Point(0, 0, 0)
        quaternion = quaternion_from_euler(0, 0, 0, 'rxyz')
        T = pose2T(pos, quaternion)
    else:
        # (pos, quaternion) = my_Baxter.getFramePose('/left_hand_camera')
        # (pos, quaternion) = my_Baxter.getFramePose('/left_gripper')
        T_base_to_arm = my_Baxter.getFramePose('/left_lower_forearm')
        T_arm_to_depth  # This is read from file
        T = T_base_to_arm.dot(T_arm_to_depth)
    return T


# Change int to str wwith specified width filled by 0
def int2str(x, width): return ("{:0"+str(width)+"d}").format(x)

# Write ndarray to file
def write_ndarray_to_file(fout, mat):
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            fout.write(str(mat[i][j])+" ")
        fout.write("\n")

# Write rgb-d camera's current pose to file
g_poses_storage=[]
def savePoseToFile(pose, ith_goalpose, clear=False):
    filename = file_folder+file_name_pose + ".txt"
    if clear==True:
        fout = open(filename,"w")
        fout.close()
        return

    # Write to file
    fout = open(filename,"a")
    fout.write("\n" + int2str(ith_goalpose,2)+"th pose: \n")
    write_ndarray_to_file(fout, pose)
    
    # Return 
    fout.close()
    g_poses_storage.append(pose)
    return

def getCloudSize(open3d_cloud):
    return np.asarray(open3d_cloud.points).shape[0]


# -- Main
if __name__ == "__main__":
    rospy.init_node('Node 1')
    DEBUG_MODE_FOR_BAXTER = rospy.get_param("~DEBUG_MODE_FOR_BAXTER")
    DEBUG_MODE_FOR_RGBDCAM = rospy.get_param("~DEBUG_MODE_FOR_RGBDCAM")

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

    # -- Speicify the goalpose_joint_angles that the Baxter needs to move to
    # 7 numbers of the 7 joint angles
    NUM_JOINTS = 7
    base_angles = [-1.0, -0.29145634174346924, 0.021859226748347282, 1.4894953966140747, -3.0480198860168457, -0.9146360158920288, -3.0269274711608887]
    def addListVal(l0, idx, val):
        l = copy(l0)
        l[idx]+=val
        return l
    goalposes_joint_angles =list()
    # -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0
    for i in range(num_goalposes):
        goalposes_joint_angles.append(
            addListVal(base_angles, 0, i*0.58/2)
        )
    assert len(goalposes_joint_angles)>=num_goalposes

    # -- Settings for debug kinect:
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
            rospy.loginfo("Node 1: sim: load cloud file: " + filename +
                          ", points = " + str(getCloudSize(open3d_cloud)))
            ros_cloud = convertCloudFromOpen3dToRos(open3d_cloud)
            rospy.loginfo("Node 1: sim: publishing cloud "+str(ith_goalpose))
            pub_sim_cloud.publish(ros_cloud)

    # Move Baxter to initial position
    rospy.sleep(1)
    rospy.loginfo("]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]")
    init_joint_angles = goalposes_joint_angles[0]
    rospy.loginfo("Node 1: Initialization. Move Baxter to init pose: "+str(init_joint_angles))
    moveBaxterToJointAngles(init_joint_angles, time_cost=3.0)
    rospy.loginfo("Node 1: Baxter reached the initial pose!\n\n")
    rospy.loginfo("[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[")

    # Move Baxter to all goal positions
    ith_goalpose = 0
    savePoseToFile(None, None, clear=True)
    while ith_goalpose < num_goalposes and not rospy.is_shutdown():
        ith_goalpose += 1
        joint_angles = goalposes_joint_angles[ith_goalpose-1]

        # Move robot to the next pose for taking picture
        rospy.loginfo("--------------------------------")
        rospy.loginfo("Node 1: {}th pos".format(ith_goalpose))
        rospy.loginfo("Node 1: Baxter is moving to pos: "+str(joint_angles))
        moveBaxterToJointAngles(joint_angles, 4.0)
        rospy.loginfo("Node 1: Baxter reached the pose!")

        # Publish the signal to node2
        rospy.loginfo("Node 1: Wait until stable for 1 more second")
        rospy.sleep(1.0)
        rospy.loginfo("Node 1: publish "+str(ith_goalpose) +
                      "th camera pose to node2")
        pose = readKinectCameraPose()
        publishPose(pose)
        savePoseToFile(pose, ith_goalpose)

        if DEBUG_MODE_FOR_RGBDCAM:  # simulate publishing a point cloud
            rospy.sleep(0.01)
            sim_pub_point_cloud(ith_goalpose)
        rospy.loginfo("--------------------------------")

        rospy.sleep(3)
        # if ith_goalpose==num_goalposes: ith_goalpose = 0

    # -- Node stops
    rospy.loginfo("!!!!! Node 1 stops.")
