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
from lib_geo_trans_ros import form_T, quaternion_to_R, toRosPose, pose2T, transXYZ
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos

# -- Message types
# from std_msgs.msg import Int32  # used for indexing the ith robot pose
from geometry_msgs.msg import Pose, Point, Quaternion
from scan3d_by_baxter.msg import T4x4


# ------------------------------------------------------------

# -- Functions: basic
# Change int to str and filled prefix with 0s
def int2str(x, width): return ("{:0"+str(width)+"d}").format(x)
    
def getCloudSize(open3d_cloud):
    return np.asarray(open3d_cloud.points).shape[0]

# -- Functions: Baxter/Camera related
class JointPosPublisher(object):
    def __init__(self, topic_endeffector_pos):
        self.pub = rospy.Publisher(topic_endeffector_pos, T4x4, queue_size=10)

    def publishPose(self, pose):
        T = pose
        # Trans to 1x16 array
        pose_1x16 = []
        for i in range(4):
            for j in range(4):
                pose_1x16 += [T[i, j]]
        self.pub.publish(pose_1x16)
        return

def readKinectCameraPose():
    T_base_to_arm = my_Baxter.getFramePose('/left_lower_forearm')
    T_arm_to_depth  # This is read from file
    T = T_base_to_arm.dot(T_arm_to_depth)
    return T

# def set_target_joint_angles():
#     target_joint_angles=[
#         [1.3456846475601196, -0.8774369955062866, 0.44907286763191223, 2.608534336090088, -1.5892040729522705, 1.0565292835235596, -3.0434179306030273,],
#         [1.138980746269226, -0.7712088227272034, 0.44370394945144653, 1.9209274053573608, -1.1696603298187256, 1.2390730381011963, -3.0476362705230713,],
#         [0.9223059415817261, -0.4134078323841095, 0.598252534866333, 0.8318010568618774, -0.9453156590461731, 1.937034249305725, -3.0476362705230713,],
#         # [0.6688156127929688, -0.3953835368156433, 0.4747670590877533, 0.5986360311508179, -0.7002622485160828, 1.928597331047058, -3.048403263092041,],
#         [0.5449466705322266, -0.3075631558895111, 0.34936413168907166, 0.39154860377311707, -0.37812626361846924, 1.914791464805603, -3.0476362705230713,],
#         [0.17679128050804138, -0.4210777282714844, 0.6170437932014465, 0.7574030160903931, -0.39154860377311707, 1.7755827903747559, -2.6714274883270264,],
#         [-0.57792729139328, -0.5146505832672119, 0.876286506652832, 1.4162477254867554, -0.03298058733344078, 1.330344796180725, -3.0480198860168457,],
#         [-0.618577778339386, -0.6373690366744995, 0.6181942820549011, 1.650563359260559, 0.302194207906723, 1.0837574005126953, -3.048403263092041,],
#         [-1.2486603260040283, -0.5836796760559082, 0.893160343170166, 1.9849711656570435, 0.06481068581342697, 0.8179952502250671, -2.9973983764648438,],
#         [-1.6106798648834229, -0.3593350052833557, 1.055378794670105, 2.2940683364868164, -0.3551165461540222, 0.5480146408081055, -2.9368062019348145,],
#     ]
#     target_joint_angles.reverse()
#     return target_joint_angles

def set_target_joint_angles():
    target_joint_angles=[
        [-1.4545972347259521, 0.40343695878982544, 1.460349678993225, 2.304422616958618, -1.2390730381011963, 1.0768544673919678, -3.0480198860168457]
        ,[-0.849825382232666, -0.35895150899887085, 1.144733190536499, 1.5957235097885132, -0.32597091794013977, 1.2973642349243164, -3.0480198860168457]
        # ,[-0.849825382232666, -0.35895150899887085, 1.144733190536499, 1.5957235097885132, -0.32597091794013977, 1.2973642349243164, -3.0480198860168457]
        ,[-0.5173349976539612, -0.5058301687240601, 0.9146360158920288, 1.584985613822937, -0.08973787724971771, 1.1888351440429688, -3.0480198860168457]
        ,[0.06404370069503784, -0.49854376912117004, 0.7117670774459839, 1.1489516496658325, -0.2105388641357422, 1.5604419708251953, -3.0468692779541016]
        ,[0.6830049753189087, -0.358568012714386, 0.10277671366930008, 0.6925923228263855, -0.060208745300769806, 1.9170924425125122, -3.0480198860168457]
        ,[0.8114758133888245, -0.520786464214325, 0.41072335839271545, 0.8248981833457947, -0.3497476279735565, 1.8860293626785278, -3.0480198860168457]
        ,[1.0270001888275146, -0.7516505718231201, 0.6688156127929688, 1.6904468536376953, -0.8095583319664001, 1.4799079895019531, -3.0476362705230713]
        ,[0.8563447594642639, -0.8893253803253174, 1.0243157148361206, 1.940102219581604, -0.7616214752197266, 1.4526797533035278, -3.047252893447876]
        ,[1.2160632610321045, -0.4141748249530792, 1.0515438318252563, 2.356194496154785, -1.3150050640106201, 1.7598594427108765, -3.0468692779541016]
        # ,[0.7405292391777039, -0.962956428527832, 1.0948787927627563, 2.177485704421997, 2.173267364501953, -1.5439516305923462, -3.0480198860168457]
        # ,[1.260932207107544, -0.9418641924858093, 1.0753204822540283, 2.452451705932617, 2.1418206691741943, -1.5136555433273315, -3.023859739303589]
   ]
    return target_joint_angles


# -- Functions: Write results to file

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


# -- Main
if __name__ == "__main__":

    # ---------------------------------------------------------------------

    rospy.init_node('Node 1')

    # -- Set Params

    file_folder = rospy.get_param("file_folder")
    file_name_pose = rospy.get_param("file_name_pose")
    file_name_index_width = rospy.get_param("file_name_index_width")

    config_folder = rospy.get_param("file_folder_config")
    file_name_T_arm_to_depth = rospy.get_param("file_name_T_arm_to_depth")
    T_arm_to_depth = np.loadtxt(config_folder+file_name_T_arm_to_depth)
    # T_baxter_to_chess = np.loadtxt(config_folder+"T_baxter_to_chess.txt")

    topic_endeffector_pos = rospy.get_param("topic_n1_to_n2")
    num_goalposes = rospy.get_param("num_goalposes")

    # -- Set Baxter
    my_Baxter = MyBaxter(['left', 'right'][0])
    my_Baxter.enableBaxter()

    # -- Set publisher: After Baxter moves to the next goalpose position,
    #   sends the pose to node2 to tell it to take the picture.
    pub = JointPosPublisher(topic_endeffector_pos)

    # ---------------------------------------------------------------------

    # Start node when pressing enter
    rospy.loginfo("\n\nWaiting for pressing 'enter' to start ...")
    raw_input("")

    # -- Speicify the goalpose_joint_angles that the Baxter needs to move to
    list_target_joint_angles = set_target_joint_angles()
    
    # -- Move Baxter to initial position
    DEBUG__I_DONT_HAVE_BAXTER=False
    if not DEBUG__I_DONT_HAVE_BAXTER:
        rospy.sleep(1)
        init_joint_angles = list_target_joint_angles[0]
        rospy.loginfo("\n\nNode 1: Initialization. Move Baxter to init pose: "+str(init_joint_angles))
        my_Baxter.moveToJointAngles(init_joint_angles, time_cost=3.0)
        rospy.loginfo("Node 1: Baxter reached the initial pose!\n\n")

    # -- Move Baxter to all goal positions
    ith_goalpose = 0
    savePoseToFile(None, None, clear=True)

    while ith_goalpose < num_goalposes and not rospy.is_shutdown():
        ith_goalpose += 1
        joint_angles = list_target_joint_angles[ith_goalpose-1]

        # Move robot to the next pose for taking picture
        if not DEBUG__I_DONT_HAVE_BAXTER:
            rospy.loginfo("\n\n------------------------------------------------------")
            rospy.loginfo("Node 1: {}th pos".format(ith_goalpose))
            rospy.loginfo("Node 1: Baxter is moving to pos: "+str(joint_angles))

            my_Baxter.moveToJointAngles(joint_angles, 4.0)
            
            # if ith_goalpose<=8:
            # elif ith_goalpose==9:
                # my_Baxter.moveToJointAngles(joint_angles, 8.0)
            # else:
                # my_Baxter.moveToJointAngles(joint_angles, 3.0)

            rospy.loginfo("Node 1: Baxter reached the pose!")

        rospy.loginfo("Node 1: Wait until stable for 1 more second")
        rospy.sleep(1.0)
        rospy.loginfo("Node 1: publish "+str(ith_goalpose) +
                      "th camera pose to node2")

        # Publish camera pose to node2
        pose = readKinectCameraPose()
        pub.publishPose(pose)
        
        # End
        savePoseToFile(pose, ith_goalpose)
        rospy.loginfo("--------------------------------")
        rospy.sleep(1)
        # if ith_goalpose==num_goalposes: ith_goalpose = 0

    # -- Node stops
    rospy.loginfo("!!!!! Node 1 stops.")
