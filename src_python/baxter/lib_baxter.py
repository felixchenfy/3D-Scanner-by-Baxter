#!/usr/bin/python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MyBaxter(object):
    def __init__(self, arm_name):
        self.limb_kinematics = baxter_kinematics(arm_name)
        self.limb = baxter_interface.Limb(arm_name)
        self.limb_interface = baxter_interface.limb.Limb(arm_name)
        self.joint_names = limb_interface.joint_names()

        def enableBaxter():
            rs = self.baxter_interface.RobotEnable(CHECK_VERSION)
            print("Enabling robot... ")
            rs.enable()
            print("Running. Ctrl-c to quit")
        enableBaxter()

   
    def getJointAngles(self):
        angles = self.limb.joint_angles().values()
        return angles

    def getEndEffectorPose(self, flag_return_euler=True):
        tmp = self.limb_kinematics.forward_position_kinematics()
        # fk returned type: [x, y, z, quat_i, quat_j, quat_k, quat_w]
        position = tmp[0:3]
        quaternion = tmp[3:]
        if flag_return_euler:
            euler=list(euler_from_quaternion(quaternion))
            return position, euler
        else:
            return position, quaternion

    def moveToJointAngles(self, joint_angles):
        print "Moving robot to joint angles : ", joint_angles
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
            quaternion = list(quaternion_from_euler(euler[0], euler[1], euler[2]))
            joint_angles = self.limb_kinematics.inverse_kinematics(pos, quaternion)
        if len_pos == 3 and len_ori == 3:
            print("Using both position and quaternion")
            quaternion = orientation
            joint_angles = self.limb_kinematics.inverse_kinematics(pos, quaternion)
        else:
            print("Wrong input!")
            joint_angles = None
            return joint_angles

        if joint_angles is None:
            print("my WARNING: IK solution not found.")
        return joint_angles


if __name__ == "__main__":
    rospy.init_node('GUI_for_Baxter')

    # Setting Baxter
    arm_name = ['left', 'right'][0]
    my_baxter = MyBaxter(arm_name)

    rospy.spin()
