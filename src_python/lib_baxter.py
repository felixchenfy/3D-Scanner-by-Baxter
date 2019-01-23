#!/usr/bin/python

import numpy as np
import sys

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import baxter_interface # For basic baxter movement
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

import actionlib # For trajectory action server
from copy import copy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from lib_geo_trans_ros import *


# This class define functions for reading states and moving the Baxter robot
class MyBaxter(object):
    def __init__(self, limb_name,
            turn_on_traj_action_server=True):
        self.limb_name = limb_name
        self.limb_kinematics = baxter_kinematics(limb_name)
        self.limb = baxter_interface.Limb(limb_name)
        self.limb_interface = baxter_interface.limb.Limb(limb_name)
        self.joint_names = self.limb_interface.joint_names()
        self.tf_listener = tf.TransformListener()

        # trajectory action client
        if turn_on_traj_action_server:
            self.traj = Trajectory(self.limb_name)
            rospy.on_shutdown(self.traj.stop)

        if 0: # optional
            self.enableRobot()

    def getJointAngles(self):
        if 0:
            # joint orders: [w0,w1,w2,e0,e1,s0,s1]
            angles = self.limb.joint_angles().values()
        else:
            # joint orders: [s0,s1,e0,e1,w0,w1,w2]
            angles = [self.limb_interface.joint_angle(joint) \
                for joint in self.limb_interface.joint_names()]
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
        A='/base'
        B=frame_name
        # print A, " -> ", B
        p_A_to_B, q_A_to_B = self.tf_listener.lookupTransform(
            A, B, rospy.Time(0))
        T_A_to_B = form_T( quaternion_matrix(q_A_to_B)[:3,:3],p_A_to_B)        
        # print "pos:", p_A_to_B
        # print "quaternion:", q_A_to_B
        return T_A_to_B

    def getCameraPose(self):
        return self.getFramePose('/'+self.limb_name+'_hand_camera')

    def moveToJointAngles(self, goal_angles, time_cost=3.0):
        if 0:     
            # joint orders: [w0,w1,w2,e0,e1,s0,s1]
            # WARNING: This method cannot control time,
            #   Baxter moves to target at whole speed, which is very unstable.
            output_angles = dict(zip(self.joint_names, goal_angles))
            print output_angles
            self.limb.set_joint_positions(output_angles)
            rospy.sleep(time_cost)
        else:
            # joint orders: [s0,s1,e0,e1,w0,w1,w2]
            # current_angles = self.getJointAngles()
            current_angles = [self.limb_interface.joint_angle(joint) for joint in self.limb_interface.joint_names()]

            self.traj.clear()
            self.traj.add_point(current_angles, 0.0)
            self.traj.add_point(goal_angles,time_cost)
            self.traj.start()
            self.traj.wait(time_cost)
            
            # -- Wait more times until reach the target
            # -- (This is not needed, the traj.wait() is defining the total time cost)
            # error_pre = 999999
            # for cnt_wait in range(10):
            #     diff_angles = np.array(self.getJointAngles())-np.array(goal_angles)
            #     error = np.linalg.norm(diff_angles)
            #     print("error: ", error)
            #     if error<0.01 or abs(error-error_pre)<0.01:
            #         break
            #     else:
            #         rospy.sleep(1.0)
            #     error_pre=error
            
            return

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

    def enableBaxter(self):
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        print("getGripperPosegetGripperPoseEnabling robot... ")
        rs.enable()
        print("Running. Ctrl-c to quit")

# The class for trajectory action client. Copied from:
# http://sdk.rethinkrobotics.com/wiki/Joint_Trajectory_Client_-_Code_Walkthrough
class Trajectory(object):
    def __init__(self, limb_name):
        ns = 'robot/limb/' + limb_name + '/'
        self.limb_name=limb_name
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [self.limb_name + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
    


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
        



if __name__ == "__main__":
    rospy.init_node('GUI_for_Baxter')

    # -- Setting Baxter
    # enableBaxter()

    # ====================================================================
    if 1:
        limb_name = ['left', 'right'][0]
        my_baxter = MyBaxter(limb_name,turn_on_traj_action_server=False)
        rospy.sleep(1.0) # sleep before query

        print("\n\n-------------------- TESTING --------------------")
        # -- test reading poses
        pose = my_baxter.getGripperPose()
        print("print end-effector pose:")
        print(pose)
        
        angles = my_baxter.getJointAngles()
        print("print joint angles:")
        print(angles)

        # -- Test read pose from tf
        if 1: 
            from lib_geo_trans_ros import pose2T
            while not rospy.is_shutdown():
                T_baxter_to_color = my_baxter.getCameraPose()
                print "T_baxter_to_color:\n", T_baxter_to_color
                rospy.sleep(3.0)

    # -- Test moving baxter to poses
    if 0:
        limb_name = ['left', 'right'][0]
        my_baxter = MyBaxter(limb_name, turn_on_traj_action_server=True)
        rospy.sleep(1.0) # sleep before query

        goal_angle1 =[-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]
        goal_angle2 =[x * 0.75 for x in goal_angle1]
        goal_angle3 =[x * 1.25 for x in goal_angle1]

        rospy.loginfo("Start moving to goal 1 ...")
        my_baxter.moveToJointAngles(goal_angle1, 7.0)
        rospy.loginfo("Reached goal 1\n")

        rospy.loginfo("Start moving to goal 2 ...")
        my_baxter.moveToJointAngles(goal_angle2, 2.0)
        rospy.loginfo("Reached goal 2\n")
        
        rospy.loginfo("Start moving to goal 3 ...")
        my_baxter.moveToJointAngles(goal_angle3, 3.0)
        rospy.loginfo("Reached goal 3\n")
    # ====================================================================
    print("Test of lib_baxter completes")
    rospy.spin()
