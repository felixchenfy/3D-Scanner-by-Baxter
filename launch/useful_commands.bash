# This is just for storing usefull command-line commands

## ===============================
# connect to Baxter
$ nmcli connection up Rethink
$ rosrun baxter_tools enable_robot.py -e

## ===============================
## my services to print Baxter pose
rosservice list |grep my
rosservice call my/PrintBaxterGripperPose
rosservice call my/PrintBaxterJointAngles

## ===============================
## tf_echo to print Baxter pose
rosrun tf tf_echo /base /left_hand_camera
rosrun tf tf_echo /base /left_gripper

## ===============================
## rostopic to print Baxter joint angles
rostopic echo -n 1 /robot/joint_states

## ===============================
# If use .launch to call service, then the code below prints out nothing .... 

# <launch>

# <node pkg="rosservice" type="rosservice" name="call_PrintBaxterGripperPose"
#     args="call my/PrintBaxterGripperPose "/>

# <node pkg="rosservice" type="rosservice" name="call_PrintBaxterJointAngles"
#     args="call my/PrintBaxterJointAngles "/>

# </launch>
