#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' What this script does:
Compute RGB-D camera's location in Baxter's forearm frame:{ 
    Read in images from two camera: Baxter left hand camera, and RGB-D camera.
    Locate chessboard in each of their image.
    Obtain relative position between two cameras.
    Obtain the pose of RGB-D camera in the Baxter's forearm frame.
}
Save result:{ 
    Display the chessboard detection result.
    Wait for user's key.
    If pressed, saved the T_forearm_to_camera (rgb-d cam) to .yaml file.
}
'''

# -- Standard
import open3d
import numpy as np
import sys, os
import cv2
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# -- ROS
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import CameraInfo
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# -- My lib
sys.path.append(PYTHON_FILE_PATH + "../../src_python")
from lib_geo_trans_ros import form_T, quaternion_to_R, toRosPose, pose2T
from lib_cam_calib import *
from lib_baxter import MyBaxter

getStrBeforeLastChar = lambda s, c: s[:(-s[::-1].find(c))]

def sub_camera_info(image_topic):
    # change topic to info: "usb_cam/image_raw" -> "usb_cam/camera_info"
    image_topic = getStrBeforeLastChar(image_topic, '/')+'camera_info'
    camera_info = rospy.wait_for_message(image_topic, topic_type=CameraInfo)
    return camera_info

def sub_camera_K_and_D(image_topic):
    camera_info = sub_camera_info(image_topic)
    K = np.array(camera_info.K).reshape((3,3))
    D = np.array(camera_info.D)
    return K, D

def sub_image(image_topic):
    ros_image = rospy.wait_for_message(image_topic, topic_type=Image)
    cv2_img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
    return cv2_img

def pubImage(publisher, cv2_image):
    publisher.publish(CvBridge().cv2_to_imgmsg(cv2_image, "bgr8"))

# -- Main
if __name__ == "__main__":
    rospy.init_node("calib_camera_pose")

    # -- Get the two cameras topic.
    # -- Note: Both cameras are using color image for locating chessboard!!!
    CAMERA_FOR_DEBUG = "usb_cam/image_raw"
    topic_cam_color = rospy.get_param("topic_baxter_left_hand_camera",
        default=CAMERA_FOR_DEBUG)
    topic_cam_depth = rospy.get_param("topic_rgbd_camera",
        default=CAMERA_FOR_DEBUG)

    # -- Set a publisher for visualization chessboard in image
    topic_pub_images_with_chessboard = rospy.get_param("~topic_pub_images_with_chessboard")
    pub = rospy.Publisher(topic_pub_images_with_chessboard, Image, queue_size=5)

    # -- Set params 
    # Camera info
    K1, D1 = sub_camera_K_and_D(topic_cam_color)
    K2, D2 = sub_camera_K_and_D(topic_cam_depth)
    print "K1: ", K1
    print "D1: ", D1
    print "K2: ", K2
    print "D2: ", D2
    cameras=[(topic_cam_color, K1, D1), (topic_cam_depth, K2, D2)]

    # Chessboard
    SQUARE_SIZE = 0.0158
    CHECKER_ROWS = 7
    CHECKER_COLS = 9

    # -- Main Loop: locate rgb-d camera pose, wait for user's keypress to save it file
    while not rospy.is_shutdown():
        Is_disp=[]
        Rs=[]
        ps=[]
        rospy.loginfo("-------------------------------------------")
        # -- Subscribe to image and locate chessboard
        for i in range(2):
            image_topic, K, D = cameras[i][0],cameras[i][1],cameras[i][2]
            I = sub_image(image_topic)
            rospy.loginfo("Subscribed a image from " + image_topic)
            res, R, p, imgpoints = getChessboardPose(I, K, D, SQUARE_SIZE , CHECKER_ROWS, CHECKER_COLS)
            
            # Draw to image
            img_display=I.copy()
            if res==True:
                drawChessboardToImage(img_display, imgpoints, CHECKER_ROWS, CHECKER_COLS)
                drawPosTextToImage(img_display, p)
                drawCoordinateToImage(img_display, R, p, K, D)

            # Return
            Is_disp.append(img_display)
            Rs.append(R)
            ps.append(p)

        # -- Display result
        I_disp = np.hstack([Is_disp[0], Is_disp[1]])
        pubImage(pub, I_disp)
        rospy.loginfo("Published a new image with chessboard result")

        rospy.sleep(0.05)

    rospy.spin()


