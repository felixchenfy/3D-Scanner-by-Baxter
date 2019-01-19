#!/usr/bin/python

import numpy as np
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Basic operations of Baxter robot
class MyBaxter(object):
    def __init__(self, arm_name):
        self.arm_name = arm_name
        self.limb_kinematics = baxter_kinematics(arm_name)
        self.limb = baxter_interface.Limb(arm_name)
        self.limb_interface = baxter_interface.limb.Limb(arm_name)
        self.joint_names = self.limb_interface.joint_names()
        self.tf_listener = tf.TransformListener()

    def getJointAngles(self):
        angles = self.limb.joint_angles().values()
        return angles

    def getGripperPose(self, flag_return_euler=True):
        tmp = self.limb_kinematics.forward_position_kinematics().tolist()
        # fk returned type: [x, y, z, quat_i, quat_j, quat_k, quat_w]
        position = tmp[0:3]
        quaternion = tmp[3:]
        if flag_return_euler:
            euler = list(euler_from_quaternion(quaternion))
            return position, euler
        else:
            return position, quaternion

    def getFramePose(self, frame_name):
        pos_list, quaternion_list = self.tf_listener.lookupTransform(
            '/base', '/left_hand_camera', rospy.Time(0))
        return pos_list, quaternion_list

    def getCameraPose(self):
        return self.getFramePose('/'+self.arm_name+'_hand_camera')

    def moveToJointAngles(self, joint_angles):
        # print "Moving robot to joint angles : ", joint_angles
        output_angles = dict(zip(self.joint_names, joint_angles))
        self.limb.set_joint_positions(output_angles)

    def computeIK(self, pos, orientation=None):
        print("Computing IK:\n")
        print(pos)
        print(orientation)
        len_pos = len(pos)
        len_ori = len(orientation)
        if len_ori == 0:
            print("Using only position")
            joint_angles = self.limb_kinematics.inverse_kinematics(pos)
        if len_pos == 3 and len_ori == 3:
            print("Using both position and euler angle")
            euler = orientation
            quaternion = list(quaternion_from_euler(
                euler[0], euler[1], euler[2]))
            joint_angles = self.limb_kinematics.inverse_kinematics(
                pos, quaternion)
        if len_pos == 3 and len_ori == 3:
            print("Using both position and quaternion")
            quaternion = orientation
            joint_angles = self.limb_kinematics.inverse_kinematics(
                pos, quaternion)
        else:
            print("Wrong input!")
            joint_angles = None
            return joint_angles

        if joint_angles is None:
            print("my WARNING: IK solution not found.")
        return joint_angles

 # A template for calling service
def callService(service_name, service_type, args=None):
    rospy.wait_for_service(service_name)
    try:
        func = rospy.ServiceProxy(service_name, service_type)
        return func(args) if args else func() # call this service
    except rospy.ServiceException, e:
        print "my ERROR: Failed to call service:", service_name
        print "The error is: ", e
        return None
        
def enableBaxter():
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("getGripperPosegetGripperPoseEnabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")


if __name__ == "__main__":
    rospy.init_node('GUI_for_Baxter')

    # -- Setting Baxter
    # enableBaxter()
    arm_name = ['left', 'right'][0]
    my_baxter = MyBaxter(arm_name)

    # -- TEST
    print("\n\n-------------------- TESTING --------------------")
    pose = my_baxter.getGripperPose()
    print("print end-effector pose:")
    print(pose)

    rospy.spin()
